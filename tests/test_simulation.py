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

            log.info("-" * 50)
            log.info("[仿真] %dHz  %s", hz, type(input_source).__name__)

            # 任务空间
            log.info("[目标] X=%+.3f  Y=%+.3f  Z=%+.3f", x, y, z)
            log.info("[实际] X=%+.3f  Y=%+.3f  Z=%+.3f  (FK⋅tendon_to_config(插值圈数))",
                     tip_achieved[0], tip_achieved[1], tip_achieved[2])

            # 驱动空间
            ik_rots = smoothed_rotations  # IK 输出的 EMA 平滑值 = 插值器目标
            rot_min, rot_max = min(ik_rots), max(ik_rots)
            interp_min, interp_max = min(rotations), max(rotations)
            log.info("[圈数] IK目标=[%+.2f, %+.2f]  "
                     "插值步进=[%+.2f, %+.2f]",
                     rot_min, rot_max, interp_min, interp_max)

            # 插值进度
            if interpolator.is_interpolating:
                max_remaining = max(
                    abs(interpolator.target[i] - interpolator.current[i])
                    for i in range(8))
                log.info("[插值] 进行中, 距目标最大剩余=%.3f 圈", max_remaining)
            else:
                log.info("[插值] 已到达目标")

            # 误差
            if last_q is not None and not all(v == 0 for v in [x, y, z]):
                ik_tip = robot.tip_position(last_q)
                err_ik = np.linalg.norm(np.array([x, y, z]) - ik_tip)
                err_now = np.linalg.norm(np.array([x, y, z]) - tip_achieved)
                log.info("[误差] IK=%.4fm  (理论精度)   "
                         "当前=%.4fm  (EMA+插值后)",
                         err_ik, err_now)

        viewer.sync()
        time.sleep(0.01)

# 清理
input_source.stop()
log.info("仿真退出")
