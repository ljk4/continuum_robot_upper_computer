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
from robot.kinematics import inverse_kinematics, MultiSectionRobot
from input.sources import create_input_source, VisionInput, TrajectoryInput, SearchInput
from robot.safety import (
    RotationInterpolator,
)
from config import serial_cfg, control_cfg, safety_cfg, vis_cfg, robot_cfg, gui_cfg
from utils.logger import setup_logger
from gui.shared_state import SharedState
from gui.gui_app import GUIThread

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
        cv2.destroyWindow("AprilTag Detection")


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

    # 4.5 启动 GUI 控制面板
    shared_state = SharedState()
    gui_thread = None
    if gui_cfg.enable_gui:
        gui_thread = GUIThread(shared_state)
        gui_thread.start()
        log.info("GUI 控制面板已启动")

    # 5. 初始化运动学模型 (用于编码器反馈→配置空间转换)
    robot = MultiSectionRobot()
    spool_circ = np.pi * robot_cfg.spool_diameter

    # 6. 初始化控制状态
    loop_count = 0
    last_fps_time = time.time()
    tx_freq = 0

    last_rotations = [0.0] * 8
    last_q = None

    # 目标位姿，初始化为零
    x = y = z = 0.0

    # 降频计数器
    vision_counter = 0
    mujoco_counter = 0
    vision_div = control_cfg.vision_update_div
    mujoco_div = control_cfg.mujoco_update_div

    # 绳长插值器（唯一平滑层，替代 EMA）
    interpolator = RotationInterpolator()
    interpolator.reset(last_rotations)

    # ACK 超时急停状态
    ack_stopped = False

    log.info("等待控制循环... (按 ESC 退出展示窗口)")

    try:
        while True:
            cycle_start = time.perf_counter()

            # ==== 每次迭代：插值 + 发送 ====
            # 用编码器反馈同步插值器当前值，闭环跟踪
            if receiver.latest_encoder is not None:
                interpolator.sync_current(receiver.latest_encoder)
            rotations = interpolator.get_next_step()
            last_rotations = rotations[:]
            sender.update_target(rotations)

            # ── ACK 超时急停（每帧检查，不等每秒日志块）──
            ack_age = time.time() - receiver.last_ack_time
            if ack_age > control_cfg.ack_timeout_sec:
                if not ack_stopped:
                    log.warning("[通信] ACK超时 %.2fs, 发送急停", ack_age)
                    serial_mgr.send(pack_stop())
                    interpolator.reset(last_rotations)
                    ack_stopped = True
            else:
                ack_stopped = False

            # ==== 每 N 次：读取输入 + IK ====
            vision_counter += 1
            if vision_counter >= vision_div:
                vision_counter = 0

                raw_rotations = None

                # ── GUI 启用时处理顶层模式 ──
                if gui_cfg.enable_gui:
                    # 检查模式切换
                    req_mode = shared_state.consume_top_mode_change()
                    if req_mode is not None:
                        log.info("[GUI] 模式切换 → %s", req_mode)
                        # 先清理旧模式的资源
                        if vis_thread is not None:
                            vis_thread.stop()
                            vis_thread.join(timeout=0.5)
                            vis_thread = None
                        if req_mode == "vision":
                            try:
                                new_src = VisionInput()
                                new_src.start()
                                input_source.stop()
                                input_source = new_src
                                shared_state.set_vision_available(True, "OK")
                                log.info("[GUI] Vision 模式已激活")
                                # 启动 OpenCV 可视化
                                if vis_cfg.enable_opencv_vis:
                                    vis_thread = VisThread(new_src)
                                    vis_thread.start()
                                    log.info("OpenCV 可视化窗口已启动")
                            except Exception as e:
                                shared_state.set_vision_available(
                                    False, str(e))
                                log.warning("[GUI] Vision 不可用: %s", e)
                        elif req_mode == "search":
                            try:
                                new_src = SearchInput()
                                new_src.start()
                                input_source.stop()
                                input_source = new_src
                                log.info("[GUI] Search 模式已激活")
                                if vis_cfg.enable_opencv_vis:
                                    vis_thread = VisThread(new_src)
                                    vis_thread.start()
                                    log.info("OpenCV 可视化窗口已启动")
                            except Exception as e:
                                log.warning("[GUI] Search 不可用: %s", e)
                        elif req_mode == "trajectory":
                            input_source.stop()
                            input_source = TrajectoryInput()
                            log.info("[GUI] Trajectory 模式已激活")
                        elif req_mode == "manual":
                            input_source.stop()
                            input_source = create_input_source()
                            input_source.start()
                            log.info("[GUI] Manual 模式已激活")

                    top_mode = shared_state.get_top_mode()

                    if top_mode == "manual":
                        # ── Manual: GUI 掌权 ──
                        gui_result = shared_state.consume_target_update()
                        zero_mode = shared_state.consume_return_to_zero()
                        if zero_mode is not None:
                            raw_rotations = [0.0] * 8
                            x = y = z = 0.0
                            last_q = [0.0] * 4
                            log.info("[GUI] 回零指令 (模式=%s)", zero_mode)
                        elif gui_result is not None:
                            mode, values = gui_result
                            if mode == "end_effector":
                                x, y, z = values
                                raw_result = inverse_kinematics(values, last_q)
                                if raw_result[0] is not None:
                                    raw_rotations, last_q = raw_result
                                else:
                                    log.warning("[GUI] IK 被拒绝")
                            elif mode == "rotations":
                                raw_rotations = values
                            elif mode == "cable_length":
                                raw_rotations = [-v / 1000.0 / spool_circ
                                               for v in values]
                            elif mode == "curvature":
                                q_gui = np.deg2rad(values)
                                tendons_gui = robot.config_to_all_tendons(q_gui)
                                raw_rotations = (-tendons_gui / spool_circ).tolist()
                                pos_gui = robot.tip_position(q_gui)
                                x, y, z = pos_gui.tolist()
                                last_q = q_gui.tolist()
                                log.info("[GUI] 曲率 (deg): %.1f %.1f %.1f %.1f",
                                         values[0], values[1], values[2], values[3])

                    else:
                        # ── Vision / Trajectory: input_source 提供目标 ──
                        direct = input_source.get_direct_rotations()
                        if direct is not None:
                            raw_rotations, q_direct, pos_direct = direct
                            if pos_direct is not None:
                                x, y, z = pos_direct
                            if q_direct is not None:
                                last_q = q_direct
                        else:
                            target = input_source.get_target()
                            if target is not None:
                                x, y, z = target
                                shared_state.set_external_target(target)
                                raw_result = inverse_kinematics(target, last_q)
                                if raw_result[0] is not None:
                                    raw_rotations, last_q = raw_result
                                else:
                                    log.warning("IK 被拒绝（不收敛或不可达），保持上次值")

                # ── 非 GUI 模式：使用 input_source ──
                else:
                    # 先尝试直接模式（跳过 IK 和位置限幅）
                    direct = input_source.get_direct_rotations()
                    if direct is not None:
                        raw_rotations, q_direct, pos_direct = direct
                        if pos_direct is not None:
                            x, y, z = pos_direct
                        if q_direct is not None:
                            last_q = q_direct
                    else:
                        # 正常 target → IK 流程
                        target = input_source.get_target()
                        if target is not None:
                            x, y, z = target

                            raw_result = inverse_kinematics(target, last_q)
                            if raw_result[0] is not None:
                                raw_rotations, last_q = raw_result
                            else:
                                log.warning("IK 被拒绝（不收敛或不可达），保持上次值")

                # 共享：插值器步进（唯一平滑层）
                # 插值器自带步长限制 (max_cable_delta/步)，
                # 不在此处做二值拒绝——否则远目标永远无法到达。
                if raw_rotations is not None:
                    interpolator.update_target(raw_rotations)

            # ==== 每 M 次：更新 MuJoCo ====
            mujoco_counter += 1
            if mujoco_counter >= mujoco_div:
                mujoco_counter = 0
                if mujoco_thread is not None:
                    if receiver.latest_encoder is not None:
                        enc = list(receiver.latest_encoder)
                        tendons = -np.array(enc) * spool_circ
                        q_actual = robot.tendon_to_config(tendons,
                                                          q0=last_q)
                        mujoco_thread.update_state(
                            q_actual, enc,
                            target_pos=[x, y, z])

            # ==== 每秒状态输出 ====
            loop_count += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                tx_freq = loop_count
                loop_count = 0
                last_fps_time = now

                log.info("-" * 50)
                log.info("[主循环] %dHz  %s", tx_freq,
                         type(input_source).__name__)
                log.info("[目标位姿] X=%+.3f Y=%+.3f Z=%+.3f", x, y, z)

                if interpolator.is_interpolating:
                    max_rem = max(abs(interpolator.target[i]
                                      - interpolator.current[i])
                                  for i in range(8))
                    log.info("[插值] 进行中, 距目标剩余=%.3f 圈", max_rem)
                else:
                    log.info("[插值] 已到达")

                arr = np.array(rotations)
                log.info("[下发圈数] 范围=[%+.3f, %+.3f]", arr.min(), arr.max())

                if receiver.latest_encoder is not None:
                    enc = receiver.latest_encoder
                    errors = np.array([
                        t - e for t, e in zip(rotations, enc)
                    ])
                    log.info("[编码器] 偏差 max=%.4f 圈", np.max(np.abs(errors)))
                else:
                    log.info("[编码器] 暂无")

                ack_age = time.time() - receiver.last_ack_time
                if ack_age > control_cfg.ack_timeout_sec:
                    log.warning("[通信] ACK超时 %.2fs", ack_age)
                else:
                    log.info("[通信] ACK %.0fms", ack_age * 1000)

                # ── 回写状态到 GUI ──
                if gui_cfg.enable_gui:
                    # 实际末端位姿（从插值器输出反算，与 MuJoCo 显示一致）
                    tendons_actual = -np.array(rotations) * spool_circ
                    q_actual = robot.tendon_to_config(tendons_actual, q0=last_q)
                    fk_actual = robot.tip_position(q_actual)
                    shared_state.set_pose(fk_actual.tolist())

                    ik_err_val = 0.0
                    if last_q is not None and not all(v == 0 for v in [x, y, z]):
                        fk_pos = robot.tip_position(np.array(last_q))
                        ik_err_val = float(np.linalg.norm(
                            np.array([x, y, z]) - fk_pos))
                    max_rot_err = 0.0
                    if receiver.latest_encoder is not None:
                        max_rot_err = float(np.max(np.abs(
                            np.array(rotations) - np.array(receiver.latest_encoder))))
                    shared_state.set_status(
                        fps=tx_freq,
                        interpolating=interpolator.is_interpolating,
                        ack_ok=not ack_stopped,
                        encoder_ok=receiver.latest_encoder is not None,
                        ik_error=ik_err_val,
                        max_rot_err=max_rot_err,
                        source_name=type(input_source).__name__,
                        active_mode="N/A",
                    )

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
        if gui_thread is not None and gui_cfg.enable_gui:
            gui_thread.stop()
            gui_thread.join(timeout=2.0)
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
