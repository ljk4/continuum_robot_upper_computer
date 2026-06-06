"""
fake_stm32.py -- STM32 下位机模拟器

在 PC 端模拟真实下位机的完整行为，用于无硬件调试。

线程架构：
    接收线程   -- 持续读取串口，解析上位机指令，回复 ACK
    PID 线程   -- 1kHz 位置伺服，驱动模拟电机跟踪目标
    反馈线程   -- 20Hz 周期发送编码器实际位置
    监控线程   -- 1Hz 打印目标/编码器/误差状态

配合虚拟串口对使用：
    main.py       打开 COM13
    fake_stm32.py 打开 COM14
    COM13 <-> COM14 通过 com0com 等虚拟串口工具相连
"""

import time
import struct
import threading
import serial

from protocol import (
    FRAME_HEADER,       # 帧头 b'\xAA\x55'
    FRAME_TAIL,         # 帧尾 b'\x0D\x0A'
    FRAME_LEN,          # 帧总长 39 字节
    CMD_TARGET,         # 命令字：目标值 0x01
    CMD_STOP,           # 命令字：急停 0x02
    CMD_QUERY,          # 命令字：查询 0x03
    CMD_FEEDBACK,       # 命令字：编码器反馈 0xA1
    CMD_ACK,            # 命令字：应答确认 0x81
    crc16_modbus,       # CRC16-Modbus 校验函数
    parse_frame         # 帧解析函数
)

from config import serial_cfg, control_cfg
from logger import setup_logger

# 初始化日志：控制台 + 文件
log = setup_logger("fake_stm32")


# =====================================================
# 全局共享状态（所有线程通过 state_lock 保护）
# =====================================================

# 目标舵机圈数（上位机下发，接收线程写入，PID线程读取）
target_rotations = [0.0] * 8

# 编码器实际圈数（PID线程写入，反馈/监控线程读取）
encoder_rotations = [0.0] * 8

# 程序运行标志（主线程退出时置 False，所有子线程随之退出）
running = True

# 线程锁：保护 target_rotations 和 encoder_rotations 的读写
state_lock = threading.Lock()

# 最近一次收到 CMD_TARGET 的时间戳，用于统计通信延迟
last_target_time = 0.0


# =====================================================
# 帧构建工具函数
# =====================================================


def build_ack():
    """构建 ACK 应答帧，通知上位机"指令已收到"

    帧格式: [帧头 2B] [CMD_ACK 1B] [空数据 32B] [CRC16 2B] [帧尾 2B]
    """

    frame = bytearray()

    # 帧头: 0xAA 0x55
    frame += FRAME_HEADER

    # 命令字: ACK = 0x81
    frame.append(CMD_ACK)

    # 数据区: 32 字节全零（ACK 帧无数据载荷）
    frame += bytes(32)

    # CRC16 校验（对帧头+命令字+数据区计算）
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)

    # 帧尾: 0x0D 0x0A
    frame += FRAME_TAIL

    return bytes(frame)


def build_feedback():
    """构建编码器反馈帧，包含 8 个舵机的当前实际圈数

    帧格式: [帧头 2B] [CMD_FEEDBACK 1B] [8×float32 32B] [CRC16 2B] [帧尾 2B]
    """

    # 线程安全地读取当前编码器值
    with state_lock:
        encoder = encoder_rotations.copy()

    frame = bytearray()

    # 帧头
    frame += FRAME_HEADER

    # 命令字: 反馈 = 0xA1
    frame.append(CMD_FEEDBACK)

    # 数据区: 8 个 float32，小端序，按 绳1~绳8 顺序排列
    frame += struct.pack("<8f", *encoder)

    # CRC16 校验
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)

    # 帧尾
    frame += FRAME_TAIL

    return bytes(frame)


# =====================================================
# PID 伺服控制线程
# =====================================================


def pid_loop():
    """1kHz 位置伺服循环

    模拟真实舵机的闭环位置跟踪行为。

    上位机下发的值是"从零位算起的绝对圈数"，
    下位机只需做标准位置伺服：error = target - current。

    控制算法: P 控制
        output += Kp * error

    保护机制:
        - 电机输出限幅: 每步最大变化量 clamp，防止阶跃
        - 到位死区: 误差小于 deadband 时停止积分，避免微小振荡
    """

    global encoder_rotations

    # 控制周期 (s): 1 / 1000Hz = 0.001s
    period = 1.0 / control_cfg.pid_hz

    # PID 参数
    Kp = control_cfg.pid_gain      # 比例增益，默认 0.01

    # 保护参数
    max_step = 0.1                 # 每步最大输出变化量 (圈/周期)
    deadband = 1e-5                # 到位死区 (圈)

    while running:

        t0 = time.perf_counter()

        with state_lock:
            # 读取目标位置（来自接收线程的最新指令）
            target = target_rotations.copy()

            for i in range(8):
                # 计算位置误差: 目标 - 实际
                error = target[i] - encoder_rotations[i]

                # 死区判断: 误差足够小则不动作
                if abs(error) < deadband:
                    continue

                # P 控制: 输出增量 = Kp × 误差
                delta = Kp * error

                # 输出限幅
                if delta > max_step:
                    delta = max_step
                elif delta < -max_step:
                    delta = -max_step

                # 更新编码器实际位置
                encoder_rotations[i] += delta

        # 精确等待下一个控制周期
        dt = time.perf_counter() - t0
        remain = period - dt
        if remain > 0:
            time.sleep(remain)


# =====================================================
# UART 接收线程
# =====================================================


class UartReceiver(threading.Thread):
    """串口接收线程

    职责:
        1. 持续从串口读取原始字节流
        2. 在缓冲区中搜索帧头 0xAA55
        3. 提取完整帧（39 字节），进行 CRC 校验和解析
        4. 根据命令字分发处理:
           - CMD_TARGET: 更新目标圈数，回复 ACK
           - CMD_STOP:   急停（锁定当前位置），回复 ACK
           - CMD_QUERY:  立即回复编码器反馈
    """

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.buffer = bytearray()

    def run(self):
        """线程主循环: 读取 -> 帧同步 -> 解析 -> 分发"""

        global target_rotations, last_target_time

        while running:

            # 读取串口缓冲区中所有可用字节
            data = self.ser.read(self.ser.in_waiting or 1)

            if not data:
                continue

            # 追加到接收缓冲区
            self.buffer.extend(data)

            # 循环处理缓冲区中所有完整帧
            while True:

                # 1) 搜索帧头 0xAA55
                idx = self.buffer.find(b"\xAA\x55")

                # 未找到帧头: 丢弃无效字节
                if idx < 0:
                    self.buffer.clear()
                    break

                # 2) 检查是否包含完整一帧
                if len(self.buffer) < idx + FRAME_LEN:
                    break

                # 3) 提取一帧数据
                frame = self.buffer[idx:idx + FRAME_LEN]
                del self.buffer[:idx + FRAME_LEN]

                # 4) CRC 校验 + 解析
                result = parse_frame(frame)

                if result is None:
                    log.warning("CRC 校验失败，丢弃该帧")
                    continue

                cmd, values = result

                # ======================================
                # CMD_TARGET: 收到目标舵机圈数
                # ======================================
                if cmd == CMD_TARGET:

                    with state_lock:
                        target_rotations = list(values)

                    last_target_time = time.time()

                    log.info("收到目标: %s",
                             [f"{v:+.4f}" for v in values])

                    # 回复 ACK
                    self.ser.write(build_ack())

                    log.debug("已回复 ACK")

                # ======================================
                # CMD_STOP: 急停指令
                # ======================================
                elif cmd == CMD_STOP:

                    log.warning("收到急停指令!")

                    with state_lock:
                        target_rotations = (
                            encoder_rotations.copy()
                        )

                    self.ser.write(build_ack())

                    log.info("急停完成，目标锁定到当前位置")

                # ======================================
                # CMD_QUERY: 状态查询
                # ======================================
                elif cmd == CMD_QUERY:

                    log.debug("收到状态查询")
                    self.ser.write(build_feedback())


# =====================================================
# 编码器反馈发送线程
# =====================================================


class FeedbackSender(threading.Thread):
    """编码器反馈发送线程

    以固定频率（默认 20Hz）向上位机发送当前编码器实际位置。
    """

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser

    def run(self):
        """线程主循环: 构建反馈帧 -> 发送 -> 等待下一周期"""

        period = 1.0 / control_cfg.feedback_hz
        frame_count = 0

        while running:

            t0 = time.perf_counter()

            frame = build_feedback()
            self.ser.write(frame)

            frame_count += 1

            # 每 100 帧记录一次（约 5 秒），避免日志过多
            if frame_count % 100 == 0:
                log.debug("已发送 %d 帧编码器反馈", frame_count)

            dt = time.perf_counter() - t0
            remain = period - dt
            if remain > 0:
                time.sleep(remain)


# =====================================================
# 状态监控线程
# =====================================================


class StatusMonitor(threading.Thread):
    """状态监控线程

    每秒打印一次下位机状态:
        - 8 个舵机的目标圈数 vs 编码器实际圈数
        - 最大跟踪误差
        - 通信延迟
    """

    def run(self):

        while running:

            time.sleep(1)

            with state_lock:
                target = target_rotations.copy()
                encoder = encoder_rotations.copy()

            # 计算跟踪误差
            errors = [
                t - e
                for t, e in zip(target, encoder)
            ]
            max_err = max(abs(e) for e in errors)

            # 计算通信延迟
            comm_delay = (
                time.time() - last_target_time
                if last_target_time > 0 else -1
            )

            # ---- 日志输出 ----
            log.info("-" * 50)
            log.info("[目标] %s",
                     "  ".join(f"绳{i+1}:{v:+.4f}"
                               for i, v in enumerate(target)))
            log.info("[编码器] %s",
                     "  ".join(f"绳{i+1}:{v:+.4f}"
                               for i, v in enumerate(encoder)))
            log.info("[误差] 最大: %.6f 圈", max_err)

            if comm_delay >= 0:
                log.info("[通信] 上次指令: %.1fs 前", comm_delay)
                if comm_delay > 1.0:
                    log.warning("超过 1s 未收到上位机指令!")
            else:
                log.info("[通信] 尚未收到指令")


# =====================================================
# 主程序入口
# =====================================================


def main():
    """启动下位机模拟器

    流程:
        1. 打开串口
        2. 启动 4 个工作线程
        3. 等待 Ctrl+C 退出
        4. 清理资源
    """

    global running

    log.info("=" * 60)
    log.info("STM32 下位机模拟器")
    log.info("=" * 60)

    # ---- 1. 打开串口 ----
    log.info("正在打开串口 %s ...", serial_cfg.port_sim)

    try:
        ser = serial.Serial(
            port=serial_cfg.port_sim,
            baudrate=serial_cfg.baudrate,
            timeout=serial_cfg.timeout
        )
    except Exception as e:
        log.error("串口打开失败: %s", e)
        return

    log.info("串口已打开")

    # ---- 2. 启动工作线程 ----
    pid_thread = threading.Thread(
        target=pid_loop, daemon=True
    )
    rx_thread = UartReceiver(ser)
    fb_thread = FeedbackSender(ser)
    monitor_thread = StatusMonitor()

    pid_thread.start()
    rx_thread.start()
    fb_thread.start()
    monitor_thread.start()

    log.info("PID 线程已启动 (%.0fHz)", control_cfg.pid_hz)
    log.info("接收线程已启动")
    log.info("反馈线程已启动 (%.0fHz)", control_cfg.feedback_hz)
    log.info("监控线程已启动 (1Hz)")
    log.info("等待上位机指令... (Ctrl+C 退出)")

    # ---- 3. 主线程阻塞等待退出信号 ----
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        log.info("检测到 Ctrl+C")

    # ---- 4. 清理资源 ----
    finally:
        running = False

        time.sleep(0.1)     # 等待子线程结束

        ser.close()
        log.info("串口已关闭")
        log.info("模拟器退出")


if __name__ == "__main__":
    main()
