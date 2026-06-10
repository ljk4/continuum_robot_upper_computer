# tests/test_simulation.py — 全流程仿真验证
#
# 与 main.py 完全相同的控制管线（从 config.py 读取配置，
# 输入源 → 限幅 → IK → EMA → 绳长检查 → 插值器），
# 区别是不下发串口，而是将插值后的圈数通过
# tendon_to_config() 转回 q，FK 得到实际末端位置，
# 在 MuJoCo 中渲染并与目标对比。
#
# 用法: python tests/test_simulation.py

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import numpy as np
import mujoco
import mujoco.viewer

from robot.kinematics import MultiSectionRobot, inverse_kinematics
from robot.safety import (
    RotationInterpolator,
)
from input.sources import create_input_source, VisionInput
from config import control_cfg, vis_cfg, robot_cfg
from vis.mujoco_model import compute_nodes, draw_scene, XML
from utils.logger import setup_logger

log = setup_logger("simulation")

# ---------- 初始化 ----------
robot = MultiSectionRobot()
spool_circ = np.pi * robot_cfg.spool_diameter

# 1. 创建输入源（与 main.py 一致，从 config.py 读模式）
input_source = create_input_source()
input_source.start()
log.info("输入源: %s", type(input_source).__name__)

# 2. 初始化控制状态（与 main.py 一致）
last_rotations = [0.0] * 8
smoothed_rotations = [0.0] * 8
last_q = None
x = y = z = 0.0

vision_counter = 0
vision_div = control_cfg.vision_update_div

ema_alpha = 0.3
ema_initialized = False

interpolator = RotationInterpolator(max_cable_delta=0.003)  # 仿真慢速展示
interpolator.reset(last_rotations)

# 3. MuJoCo
model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

loop_count = 0
last_log_time = time.time()
frame_count = 0

q_achieved = np.zeros(4)  # 当前显示姿态, 每帧由 tendon_to_config 平滑更新


def _section1_tip(q_arr):
    """计算第一部分（n 段 PCC）末端在基坐标系下的位置"""
    T = np.eye(4)
    T1 = robot.sec1.transform(q_arr[0], q_arr[1])
    for _ in range(robot.n):
        T = T @ T1
    return T[:3, 3]


log.info("仿真开始 (按关闭窗口退出)")

# ---------- 主循环 ----------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # ==== 插值步进（与 main.py 一致）====
        rotations = interpolator.get_next_step()
        last_rotations = rotations[:]

        # 驱动空间 → 配置空间 (tendon_to_config 替代实物机器人)
        # 用上一帧姿态作初值，保证平滑追踪插值步进
        tendons = -np.array(rotations) * spool_circ
        q_achieved = robot.tendon_to_config(tendons, q0=q_achieved)

        # 配置空间 → 任务空间
        tip_achieved = robot.tip_position(q_achieved)

        # ==== 每 N 次: 读取输入 + IK（与 main.py 一致）====
        vision_counter += 1
        if vision_counter >= vision_div:
            vision_counter = 0

            raw_rotations = None

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

                    raw_result = inverse_kinematics(target, last_q)
                    if raw_result[0] is not None:
                        raw_rotations, last_q = raw_result

            # 共享: EMA + 插值器（插值器自带步长限制，不二值拒绝）
            if raw_rotations is not None:
                if not ema_initialized:
                    smoothed_rotations = list(raw_rotations)
                    ema_initialized = True
                else:
                    for i in range(8):
                        smoothed_rotations[i] = (
                            ema_alpha * raw_rotations[i]
                            + (1 - ema_alpha) * smoothed_rotations[i])

                interpolator.update_target(smoothed_rotations)

        # ==== MuJoCo 渲染 ====
        mujoco.mj_forward(model, data)
        draw_scene(viewer, q_achieved,
                   target_pos=[x, y, z] if not all(v == 0 for v in [x, y, z])
                              or input_source.get_target() is not None else None,
                   rotations=rotations)

        # ==== 每秒状态输出 ====
        loop_count += 1
        frame_count += 1
        now = time.time()
        if now - last_log_time >= 1.0:
            hz = loop_count
            loop_count = 0
            last_log_time = now

            # 预计算第一部分末端
            sec1_tip_achieved = _section1_tip(q_achieved)
            ik_q = last_q  # IK 求解得到的配置空间目标
            sec1_tip_target = _section1_tip(ik_q) if ik_q is not None else None

            ik_rots = smoothed_rotations  # IK 输出的 EMA 平滑值 = 插值器目标

            log.info("=" * 60)
            log.info("[仿真] %dHz  %s", hz, type(input_source).__name__)

            # ================================================================
            # 任务空间 (Task Space) — 笛卡尔位置
            # ================================================================
            log.info("── 任务空间 (Task Space) ──")
            log.info("  目标 末端:        X=%+8.4f  Y=%+8.4f  Z=%+8.4f", x, y, z)
            if sec1_tip_target is not None:
                log.info("  目标 第一部分末端: X=%+8.4f  Y=%+8.4f  Z=%+8.4f",
                         sec1_tip_target[0], sec1_tip_target[1], sec1_tip_target[2])
            log.info("  实际 第一部分末端: X=%+8.4f  Y=%+8.4f  Z=%+8.4f",
                     sec1_tip_achieved[0], sec1_tip_achieved[1], sec1_tip_achieved[2])
            log.info("  实际 末端:        X=%+8.4f  Y=%+8.4f  Z=%+8.4f  (FK·tendon_to_config)",
                     tip_achieved[0], tip_achieved[1], tip_achieved[2])

            # ================================================================
            # 配置空间 (Configuration Space) — 弯曲角
            # ================================================================
            log.info("── 配置空间 (Configuration Space) ──")
            log.info("  %-10s  %8s  %8s  %8s  %8s", "", "θ1(°)", "φ1(°)", "θ2(°)", "φ2(°)")
            if ik_q is not None:
                log.info("  %-10s  %+8.2f  %+8.2f  %+8.2f  %+8.2f",
                         "IK目标", np.rad2deg(ik_q[0]), np.rad2deg(ik_q[1]),
                         np.rad2deg(ik_q[2]), np.rad2deg(ik_q[3]))
            log.info("  %-10s  %+8.2f  %+8.2f  %+8.2f  %+8.2f",
                     "当前实际", np.rad2deg(q_achieved[0]), np.rad2deg(q_achieved[1]),
                     np.rad2deg(q_achieved[2]), np.rad2deg(q_achieved[3]))

            # ================================================================
            # 驱动空间 (Actuation Space) — 舵机圈数
            # ================================================================
            log.info("── 驱动空间 (Actuation Space / 圈数) ──")
            log.info("  %-12s  %8s  %8s  %8s  %8s", "第一部分绳", "#1", "#2", "#3", "#4")
            log.info("  %-12s  %+8.3f  %+8.3f  %+8.3f  %+8.3f",
                     "IK目标", ik_rots[0], ik_rots[1], ik_rots[2], ik_rots[3])
            log.info("  %-12s  %+8.3f  %+8.3f  %+8.3f  %+8.3f",
                     "插值步进", rotations[0], rotations[1], rotations[2], rotations[3])
            log.info("  %-12s  %8s  %8s  %8s  %8s", "第二部分绳", "#5", "#6", "#7", "#8")
            log.info("  %-12s  %+8.3f  %+8.3f  %+8.3f  %+8.3f",
                     "IK目标", ik_rots[4], ik_rots[5], ik_rots[6], ik_rots[7])
            log.info("  %-12s  %+8.3f  %+8.3f  %+8.3f  %+8.3f",
                     "插值步进", rotations[4], rotations[5], rotations[6], rotations[7])

            # ================================================================
            # 插值进度
            # ================================================================
            if interpolator.is_interpolating:
                # 分部分显示剩余
                rem_sec1 = max(
                    abs(interpolator.target[i] - interpolator.current[i])
                    for i in range(4))
                rem_sec2 = max(
                    abs(interpolator.target[i] - interpolator.current[i])
                    for i in range(4, 8))
                log.info("── 插值进度: 进行中, 第一部分最大剩余=%.4f 圈, "
                         "第二部分最大剩余=%.4f 圈", rem_sec1, rem_sec2)
            else:
                log.info("── 插值进度: 已到达目标")

            # ================================================================
            # 误差
            # ================================================================
            if ik_q is not None and not all(v == 0 for v in [x, y, z]):
                ik_tip = robot.tip_position(ik_q)
                err_ik = np.linalg.norm(np.array([x, y, z]) - ik_tip)
                err_now = np.linalg.norm(np.array([x, y, z]) - tip_achieved)
                # 分部分误差
                if sec1_tip_target is not None:
                    err_sec1 = np.linalg.norm(sec1_tip_target - sec1_tip_achieved)
                    log.info("── 误差 ──")
                    log.info("  IK理论误差:       %.4f m  (目标 vs IK·FK 末端)", err_ik)
                    log.info("  当前实际误差:     %.4f m  (目标 vs 插值后FK 末端)", err_now)
                    log.info("  第一部分追踪误差: %.4f m  (目标第一部分末端 vs 实际)", err_sec1)
                else:
                    log.info("── 误差 ──")
                    log.info("  IK理论误差:   %.4f m  (目标 vs IK·FK)", err_ik)
                    log.info("  当前实际误差: %.4f m  (目标 vs 插值后FK)", err_now)

        viewer.sync()
        time.sleep(0.01)

# 清理
input_source.stop()
log.info("仿真退出")
