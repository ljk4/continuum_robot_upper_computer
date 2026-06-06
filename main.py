# main.py -- 上位机主控制循环

import time
import threading
import logging
import numpy as np
import cv2

from serial_manager import SerialManager
from sender import SenderThread
from receiver import ReceiverThread
from vision_tracker import VisionTracker, draw_detection
from ik_solver import inverse_kinematics
from config import serial_cfg, control_cfg
from logger import setup_logger

# 初始化日志：控制台 + 文件
log = setup_logger("main")


# =====================================================
# 视觉采集线程
# =====================================================

class VisionThread(threading.Thread):
    """独立线程持续采集摄像头帧，避免阻塞主循环

    主循环直接读取 latest_pose，无需等待摄像头帧读取（~20ms）
    """

    def __init__(self):
        super().__init__(daemon=True)

        self.tracker = None
        self.latest_pose = None    # 最新的有效位姿 [x, y, z]，未检测到时为 None
        self.latest_result = None  # 最新的完整检测结果（含 frame）
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        """持续采集，更新 latest_pose"""

        try:
            self.tracker = VisionTracker()
            log.info("视觉线程: 摄像头已打开")
        except Exception as e:
            log.error("视觉线程: 摄像头打开失败: %s", e)
            return

        while self.running:
            result = self.tracker.get_pose()

            if result is None:
                continue

            pose = result.get("pose")

            with self.lock:
                self.latest_pose = pose
                self.latest_result = result

    def get_pose(self):
        """线程安全地读取最新位姿"""

        with self.lock:
            return self.latest_pose

    def get_result(self):
        """线程安全地读取最新完整检测结果"""

        with self.lock:
            return self.latest_result

    def stop(self):
        self.running = False
        if self.tracker:
            try:
                self.tracker.release()
            except Exception:
                pass


class VisThread(threading.Thread):
    """独立线程运行 OpenCV 窗口展示

    cv2.imshow + cv2.waitKey 会阻塞调用线程，
    必须在独立线程中运行，避免拖慢主控制循环。
    """

    def __init__(self, vision_thread):
        super().__init__(daemon=True)

        self.vision_thread = vision_thread
        self.running = True

    def run(self):

        while self.running:

            result = self.vision_thread.get_result()

            if result is not None:
                vis_frame = draw_detection(result)
                cv2.imshow("AprilTag Detection", vis_frame)

            # waitKey 必须调用，否则窗口无响应
            # 返回值 & 0xFF == 27 表示按了 ESC
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                self.running = False
                break

    def stop(self):
        self.running = False


def main():
    """主控制循环

    流程:
        1. 打开串口，启动视觉/发送/接收线程
        2. 50Hz 循环: 读取视觉结果 -> 逆运动学 -> 发送目标
        3. 每秒打印一次状态
        4. Ctrl+C 退出时清理资源
    """

    log.info("=" * 60)
    log.info("绳驱动并联机器人上位机控制系统")
    log.info("=" * 60)

    # ---- 1. 打开串口 ----
    log.info("正在打开串口 %s ...", serial_cfg.port_main)

    try:
        serial_mgr = SerialManager()
    except Exception as e:
        log.error("串口打开失败: %s", e)
        return

    log.info("串口已打开")

    # ---- 2. 启动线程 ----
    vision_thread = VisionThread()
    vision_thread.start()

    sender = SenderThread(serial_mgr)
    receiver = ReceiverThread(serial_mgr)

    sender.start()
    receiver.start()

    vis_thread = VisThread(vision_thread)
    vis_thread.start()

    log.info("视觉线程已启动")
    log.info("展示线程已启动")
    log.info("发送线程已启动 (%.0fHz)", control_cfg.send_hz)
    log.info("接收线程已启动")

    log.info("等待控制循环... (按 ESC 退出展示窗口)")

    # ---- 3. 主控制循环 ----
    loop_count = 0
    last_fps_time = time.time()
    tx_freq = 0

    # 上次有效的舵机圈数（未检测到标记时保持不动）
    last_rotations = [0.0] * 8

    # IK 降频：每 N 个主循环迭代计算一次 IK
    ik_interval = 5        # 50Hz / 5 = 10Hz IK 更新
    ik_counter = 0

    # EMA 低通滤波：平滑 IK 输出，抑制抖动
    ema_alpha = 0.3        # 滤波系数 (0~1)，越小越平滑
    smoothed_rotations = [0.0] * 8

    # IK 初始猜测（上次解），加速收敛
    last_q = None

    try:

        while True:

            cycle_start = time.perf_counter()

            # ---- 3.1 读取视觉结果 ----
            pose = vision_thread.get_pose()

            # ---- 3.2 逆运动学（降频运行） ----
            ik_counter += 1

            if pose is not None:
                x, y, z = pose

                if ik_counter >= ik_interval:
                    ik_counter = 0
                    raw_rotations, last_q = inverse_kinematics(pose, last_q)

                    # EMA 低通滤波
                    for i in range(8):
                        smoothed_rotations[i] = (
                            ema_alpha * raw_rotations[i]
                            + (1 - ema_alpha) * smoothed_rotations[i]
                        )

                    last_rotations = smoothed_rotations[:]

                    log.debug("舵机圈数: %s",
                              [f"{r:+.4f}" for r in smoothed_rotations])

                rotations = last_rotations
            else:
                x, y, z = 0.0, 0.0, 0.0
                rotations = last_rotations
                log.debug("未检测到标记，保持上次目标")

            # ---- 3.3 更新发送目标 ----
            sender.update_target(rotations)

            # ---- 3.4 每秒状态输出 ----
            loop_count += 1
            now = time.time()

            if now - last_fps_time >= 1.0:

                tx_freq = loop_count
                loop_count = 0
                last_fps_time = now

                detected = pose is not None

                # 状态
                log.info("[状态] 主循环: %dHz  视觉: %s",
                         tx_freq, "已检测" if detected else "未检测")

                # 位姿
                log.info("[位姿] X=%.3f  Y=%.3f  Z=%.3f", x, y, z)

                # 舵机圈数（详细数据写入文件，控制台只显示摘要）
                arr = np.array(rotations)
                log.info("[目标] 范围=[%.4f, %.4f] 圈", arr.min(), arr.max())
                for i, rot in enumerate(rotations):
                    log.debug("  绳%d: %+.4f 圈", i + 1, rot)

                # 编码器反馈
                if receiver.latest_encoder is not None:
                    enc = receiver.latest_encoder
                    errors = np.array([
                        t - e for t, e in zip(rotations, enc)
                    ])
                    log.info("[误差] 最大=%.6f 圈",
                             np.max(np.abs(errors)))
                    for i, val in enumerate(enc):
                        log.debug("  编码器%d: %+.4f 圈", i + 1, val)
                else:
                    log.info("[编码器] 暂无反馈数据")

                # 通信状态
                ack_age = time.time() - receiver.last_ack_time
                log.info("[通信] 上次ACK间隔: %.3fs", ack_age)

                if ack_age > control_cfg.ack_timeout_sec:
                    log.warning("下位机通信超时! (%.3fs)", ack_age)
                else:
                    log.debug("通信正常")

            # ---- 3.5 周期耗时检查 ----
            cycle_time = time.perf_counter() - cycle_start

            if cycle_time > control_cfg.slow_loop_warn_sec:
                log.warning("主循环过慢: %.2fms", cycle_time * 1000)

            # ---- 3.6 50Hz 定时 ----
            sleep_time = (1.0 / control_cfg.main_loop_hz) - cycle_time

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        log.info("检测到 Ctrl+C")

    except Exception as e:
        log.error("严重错误: %s: %s", type(e).__name__, e)
        import traceback
        log.error(traceback.format_exc())

    # ---- 4. 清理资源 ----
    finally:
        log.info("正在停止...")

        # 先停止所有线程标志
        vis_thread.stop()
        vision_thread.stop()
        sender.stop()
        receiver.stop()

        # 关串口，强制中断阻塞在 I/O 上的线程
        serial_mgr.close()
        log.info("串口已关闭")

        # 等待线程退出（超时 2 秒）
        sender.join(timeout=2.0)
        receiver.join(timeout=2.0)

        # 最后销毁 OpenCV 窗口（此时 VisThread 已停止）
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        log.info("系统退出")


if __name__ == "__main__":
    main()
