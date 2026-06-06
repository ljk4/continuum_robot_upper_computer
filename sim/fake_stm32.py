"""
sim/fake_stm32.py -- STM32 下位机模拟器

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

from comm.protocol import (
    FRAME_HEADER,
    FRAME_TAIL,
    FRAME_LEN,
    CMD_TARGET,
    CMD_STOP,
    CMD_QUERY,
    CMD_FEEDBACK,
    CMD_ACK,
    crc16_modbus,
    parse_frame,
)
from config import serial_cfg, control_cfg
from utils.logger import setup_logger

log = setup_logger("fake_stm32")

# 全局共享状态
target_rotations = [0.0] * 8
encoder_rotations = [0.0] * 8
running = True
state_lock = threading.Lock()
last_target_time = 0.0


def build_ack():
    frame = bytearray()
    frame += FRAME_HEADER
    frame.append(CMD_ACK)
    frame += bytes(32)
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    frame += FRAME_TAIL
    return bytes(frame)


def build_feedback():
    with state_lock:
        encoder = encoder_rotations.copy()

    frame = bytearray()
    frame += FRAME_HEADER
    frame.append(CMD_FEEDBACK)
    frame += struct.pack("<8f", *encoder)
    crc = crc16_modbus(frame)
    frame += struct.pack("<H", crc)
    frame += FRAME_TAIL
    return bytes(frame)


def pid_loop():
    """1kHz 比例控制循环"""
    global encoder_rotations

    period = 1.0 / control_cfg.pid_hz
    Kp = control_cfg.pid_gain
    max_step = 0.1
    deadband = 1e-5

    while running:
        t0 = time.perf_counter()

        with state_lock:
            target = target_rotations.copy()
            for i in range(8):
                error = target[i] - encoder_rotations[i]
                if abs(error) < deadband:
                    continue
                delta = Kp * error
                if delta > max_step:
                    delta = max_step
                elif delta < -max_step:
                    delta = -max_step
                encoder_rotations[i] += delta

        dt = time.perf_counter() - t0
        remain = period - dt
        if remain > 0:
            time.sleep(remain)


class UartReceiver(threading.Thread):
    """串口接收线程"""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.buffer = bytearray()

    def run(self):
        global target_rotations, last_target_time

        while running:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue

            self.buffer.extend(data)

            while True:
                idx = self.buffer.find(b"\xAA\x55")
                if idx < 0:
                    self.buffer.clear()
                    break
                if len(self.buffer) < idx + FRAME_LEN:
                    break

                frame = self.buffer[idx:idx + FRAME_LEN]
                del self.buffer[:idx + FRAME_LEN]

                result = parse_frame(frame)
                if result is None:
                    log.warning("CRC 校验失败，丢弃该帧")
                    continue

                cmd, values = result

                if cmd == CMD_TARGET:
                    with state_lock:
                        target_rotations = list(values)
                    last_target_time = time.time()
                    log.info("收到目标: %s",
                             [f"{v:+.4f}" for v in values])
                    self.ser.write(build_ack())
                    log.debug("已回复 ACK")

                elif cmd == CMD_STOP:
                    log.warning("收到急停指令!")
                    with state_lock:
                        target_rotations = encoder_rotations.copy()
                    self.ser.write(build_ack())
                    log.info("急停完成，目标锁定到当前位置")

                elif cmd == CMD_QUERY:
                    log.debug("收到状态查询")
                    self.ser.write(build_feedback())


class FeedbackSender(threading.Thread):
    """编码器反馈发送线程（20Hz）"""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser

    def run(self):
        period = 1.0 / control_cfg.feedback_hz
        frame_count = 0

        while running:
            t0 = time.perf_counter()
            frame = build_feedback()
            self.ser.write(frame)
            frame_count += 1

            if frame_count % 100 == 0:
                log.debug("已发送 %d 帧编码器反馈", frame_count)

            dt = time.perf_counter() - t0
            remain = period - dt
            if remain > 0:
                time.sleep(remain)


class StatusMonitor(threading.Thread):
    """状态监控线程（1Hz）"""

    def run(self):
        while running:
            time.sleep(1)

            with state_lock:
                target = target_rotations.copy()
                encoder = encoder_rotations.copy()

            errors = [t - e for t, e in zip(target, encoder)]
            max_err = max(abs(e) for e in errors)

            comm_delay = (
                time.time() - last_target_time
                if last_target_time > 0 else -1
            )

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


def main():
    global running

    log.info("=" * 60)
    log.info("STM32 下位机模拟器")
    log.info("=" * 60)

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

    pid_thread = threading.Thread(target=pid_loop, daemon=True)
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

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("检测到 Ctrl+C")
    finally:
        running = False
        time.sleep(0.1)
        ser.close()
        log.info("串口已关闭")
        log.info("模拟器退出")


if __name__ == "__main__":
    main()
