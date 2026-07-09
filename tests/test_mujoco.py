# tests/test_mujoco.py -- MuJoCo 独立可视化演示：IK 追踪目标

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import mujoco
import mujoco.viewer
import numpy as np

from robot.kinematics import MultiSectionRobot
from config import robot_cfg
from vis.mujoco_model import compute_nodes, draw_scene, XML
from utils.logger import setup_logger

log = setup_logger("test_mujoco")

robot = MultiSectionRobot()
spool_circ = np.pi * robot_cfg.spool_diameter

# 两个可达目标：由 FK 生成，保证在 workspace 内
targets = [
    np.array([0.25, 0.15, 0.90]),
    np.array([0.40, -0.20, 0.70]),
]

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

# 每个目标缓存各自的 IK 解（冗余机器人有多解，复用保证一致性）
q_cache = {}   # key: tuple(target) → q
q = np.zeros(4)
rots = np.zeros(robot_cfg.num_cables)
ik_success = False

with mujoco.viewer.launch_passive(model, data) as viewer:
    frame_count = 0
    current_idx = -1
    while viewer.is_running():
        # 每 3 秒切换目标
        target_idx = (frame_count // 180) % 2
        new_target = targets[target_idx]

        # 目标切换时重算 IK，用该目标缓存的解作为初始猜测
        if target_idx != current_idx:
            current_idx = target_idx
            q0 = q_cache.get(tuple(new_target))  # 复用缓存，首次为 None
            result = robot.inverse_kinematics(new_target, q0=q0)
            if result is not None:
                q = result
                q_cache[tuple(new_target)] = q   # 缓存供下次复用
                tendons = robot.config_to_all_tendons(q)
                rots = -tendons / spool_circ
                ik_success = True
            else:
                ik_success = False

        mujoco.mj_forward(model, data)

        draw_scene(viewer, q, target_pos=new_target, rotations=rots)

        # 每秒输出三空间参数
        frame_count += 1
        if frame_count % 60 == 0:
            nodes = compute_nodes(q)
            tip = nodes[-1]
            print(f"[驱动] 圈数: {' '.join(f'{r:+.3f}' for r in rots)}")
            print(f"[配置] θ1={np.rad2deg(q[0]):5.1f}° "
                  f"φ1={np.rad2deg(q[1]):6.1f}°  "
                  f"θ2={np.rad2deg(q[2]):5.1f}° "
                  f"φ2={np.rad2deg(q[3]):6.1f}°")
            print(f"[任务] 末端: X={tip[0]:+.4f} Y={tip[1]:+.4f} Z={tip[2]:+.4f}")
            err = np.linalg.norm(new_target - tip)
            status = "OK" if ik_success else "FAIL"
            print(f"[任务] 目标: X={new_target[0]:+.4f} Y={new_target[1]:+.4f} "
                  f"Z={new_target[2]:+.4f}  ||误差||={err:.4f}  [{status}]")
            print("-" * 50)

        viewer.sync()
        time.sleep(0.01)
