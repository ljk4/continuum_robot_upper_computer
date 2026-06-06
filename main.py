# main.py -- 上位机主控制循环

import time
import threading
import numpy as np
import cv2

from comm.serial_mgr import SerialManager
from comm.sender import SenderThread
from comm.receiver import ReceiverThread
from comm.protocol import pack_stop
from vision.tracker import draw_detection
from robot.kinematics import inverse_kinematics
from input.sources import create_input_source, VisionInput
from robot.safety import (
    limit_position_change,
    RotationInterpolator,
)
from config import serial_cfg, control_cfg, safety_cfg, vis_cfg
from utils.logger import setup_logger

log = setup_logger("main")


class VisThread(threading.Thread):
    """OpenCV 展示线程"""

    def __init__(self, vision_input):
        super().__init__(daemon=True)
        self.vision_input = vision_input
        self.running = True

    def run(self):
        while self.running:
            result = self.vision_input.get_result()
            if result is not None:
                vis_frame = draw_detection(result)
                cv2.imshow("AprilTag Detection", vis_frame)

            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                self.running = False
                break

    def stop(self):
        self.running = False


def main():
    log.info("=" * 60)
    log.info("绳驱动并联机器人上位机控制系统")
    log.info("=" * 60)

    # 1. 创建输入源
    input_source = create_input_source()
    input_source.start()
    log.info("输入源: %s", type(input_source).__name__)

    # 2. 打开串口
    log.info("正在打开串口 %s ...", serial_cfg.port_main)
    try:
        serial_mgr = SerialManager()
    except Exception as e:
        log.error("串口打开失败: %s", e)
        input_source.stop()
        return

    log.info("串口已打开")

    # 3. 启动通信线程
    sender = SenderThread(serial_mgr)
    receiver = ReceiverThread(serial_mgr)
    sender.start()
    receiver.start()
    log.info("发送线程已启动 (%.0fHz)", control_cfg.send_hz)
    log.info("接收线程已启动")

    # 4. 条件启动可视化
    vis_thread = None
    if vis_cfg.enable_opencv_vis and isinstance(input_source, VisionInput):
        vis_thread = VisThread(input_source)
        vis_thread.start()
        log.info("OpenCV 可视化线程已启动")

    mujoco_thread = None
    if vis_cfg.enable_mujoco_vis:
        from vis.mujoco_vis import MuJoCoVisThread
        mujoco_thread = MuJoCoVisThread()
        mujoco_thread.start()
        log.info("MuJoCo 可视化线程已启动")

    # 5. 初始化控制状态
    loop_count = 0
    last_fps_time = time.time()
    tx_freq = 0

    last_rotations = [0.0] * 8
    last_target = [0.0, 0.0, 0.0]
    smoothed_rotations = [0.0] * 8
    last_q = None

    # 目标位姿，初始化为零
    x = y = z = 0.0

    # 降频计数器
    vision_counter = 0
    mujoco_counter = 0
    vision_div = control_cfg.vision_update_div
    mujoco_div = control_cfg.mujoco_update_div

    # EMA 低通滤波
    ema_alpha = 0.3
    ema_initialized = False

    # 绳长插值器
    interpolator = RotationInterpolator()
    interpolator.reset(last_rotations)

    # ACK 超时急停状态
    ack_stopped = False

    log.info("等待控制循环... (按 ESC 退出展示窗口)")

    try:
        while True:
            cycle_start = time.perf_counter()

            # ==== 每次迭代：插值 + 发送 ====
            rotations = interpolator.get_next_step()
            last_rotations = rotations[:]
            sender.update_target(rotations)

            # ==== 每 N 次：读取输入 + IK ====
            vision_counter += 1
            if vision_counter >= vision_div:
                vision_counter = 0

                raw_rotations = None

                # 先尝试直接模式（跳过 IK 和位置限幅）
                direct = input_source.get_direct_rotations()
                if direct is not None:
                    raw_rotations, q_direct, pos_direct = direct
                    if pos_direct is not None:
                        x, y, z = pos_direct
                    if q_direct is not None:
                        last_q = q_direct
                    log.debug("直接模式: 圈数=%s",
                              [f"{r:+.4f}" for r in raw_rotations])

                else:
                    # 正常 target → IK 流程
                    target = input_source.get_target()
                    if target is not None:
                        x, y, z = target

                        target = limit_position_change(
                            target, last_target,
                            safety_cfg.max_position_change
                        )

                        raw_result = inverse_kinematics(target, last_q)
                        if raw_result[0] is not None:
                            raw_rotations, last_q = raw_result
                            last_target = target[:]
                        else:
                            log.debug("IK 被拒绝（不收敛或不可达），保持上次值")

                # 共享：EMA + 平滑 + 插值器
                # 插值器自带步长限制 (max_cable_delta/步)，
                # 不在此处做二值拒绝——否则远目标永远无法到达。
                if raw_rotations is not None:
                    if not ema_initialized:
                        smoothed_rotations = list(raw_rotations)
                        ema_initialized = True
                    else:
                        for i in range(8):
                            smoothed_rotations[i] = (
                                ema_alpha * raw_rotations[i]
                                + (1 - ema_alpha) * smoothed_rotations[i]
                            )

                    interpolator.update_target(smoothed_rotations)
                    log.debug("舵机圈数: %s",
                              [f"{r:+.4f}" for r in smoothed_rotations])

            # ==== 每 M 次：更新 MuJoCo ====
            mujoco_counter += 1
            if mujoco_counter >= mujoco_div:
                mujoco_counter = 0
                if mujoco_thread is not None and last_q is not None:
                    mujoco_thread.update_state(
                        last_q, rotations,
                        target_pos=[x, y, z])

            # ==== 每秒状态输出 ====
            loop_count += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                tx_freq = loop_count
                loop_count = 0
                last_fps_time = now

                log.info("[状态] 主循环: %dHz  输入: %s",
                         tx_freq, type(input_source).__name__)
                log.info("[位姿] X=%.3f  Y=%.3f  Z=%.3f", x, y, z)

                arr = np.array(rotations)
                log.info("[目标] 范围=[%.4f, %.4f] 圈", arr.min(), arr.max())

                if interpolator.is_interpolating:
                    log.info("[插值] 进行中")

                if receiver.latest_encoder is not None:
                    enc = receiver.latest_encoder
                    errors = np.array([
                        t - e for t, e in zip(rotations, enc)
                    ])
                    log.info("[误差] 最大=%.6f 圈", np.max(np.abs(errors)))
                else:
                    log.info("[编码器] 暂无反馈数据")

                ack_age = time.time() - receiver.last_ack_time
                log.info("[通信] 上次ACK间隔: %.3fs", ack_age)

                if ack_age > control_cfg.ack_timeout_sec:
                    log.warning("下位机通信超时! (%.3fs)", ack_age)
                    if not ack_stopped:
                        log.warning("发送急停帧并锁定当前位置")
                        serial_mgr.send(pack_stop())
                        interpolator.reset(last_rotations)
                        ack_stopped = True
                else:
                    ack_stopped = False

            # ==== 周期耗时检查 ====
            cycle_time = time.perf_counter() - cycle_start
            if cycle_time > control_cfg.slow_loop_warn_sec:
                log.warning("主循环过慢: %.2fms", cycle_time * 1000)

            # ==== 定时 ====
            sleep_time = (1.0 / control_cfg.main_loop_hz) - cycle_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        log.info("检测到 Ctrl+C")

    except Exception as e:
        log.error("严重错误: %s: %s", type(e).__name__, e)
        import traceback
        log.error(traceback.format_exc())

    finally:
        log.info("正在停止...")

        input_source.stop()
        if vis_thread is not None:
            vis_thread.stop()
        if mujoco_thread is not None:
            mujoco_thread.stop()
        sender.stop()
        receiver.stop()

        serial_mgr.close()
        log.info("串口已关闭")

        sender.join(timeout=2.0)
        receiver.join(timeout=2.0)

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        log.info("系统退出")


if __name__ == "__main__":
    main()
