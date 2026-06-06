# vis/mujoco_model.py -- MuJoCo 场景定义、节点计算、共享绘制

import numpy as np

from robot.kinematics import MultiSectionRobot
from config import robot_cfg
import mujoco

_robot = MultiSectionRobot()

XML = """
<mujoco model="continuum">
    <option gravity="0 0 0"/>
    <visual>
        <headlight diffuse="1 1 1"/>
    </visual>
    <worldbody>
        <geom type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
        <body name="base">
            <geom type="cylinder" size="0.04 0.02" rgba="0.4 0.4 0.4 1"/>
        </body>
    </worldbody>
</mujoco>
"""


def compute_nodes(q):
    """计算所有 PCC 段节点的 3D 坐标

    参数:
        q: [theta1, phi1, theta2, phi2] (rad)

    返回:
        (n+m+1, 3) 节点坐标数组，第一个节点在原点
    """
    theta1, phi1, theta2, phi2 = q
    T = np.eye(4)
    nodes = [np.zeros(3)]

    T1 = _robot.sec1.transform(theta1, phi1)
    for _ in range(_robot.n):
        T = T @ T1
        nodes.append(T[:3, 3].copy())

    T2 = _robot.sec2.transform(theta2, phi2)
    for _ in range(_robot.m):
        T = T @ T2
        nodes.append(T[:3, 3].copy())

    return np.array(nodes)


def draw_scene(viewer, q, target_pos=None, rotations=None):
    """在 MuJoCo viewer 中绘制完整场景

    绘制内容：
    - 基座坐标系 (X=红, Y=绿, Z=蓝)
    - 机器人节点 (蓝球) + 骨架 (青色胶囊)
    - 目标点 (红色大球，如提供)
    - 末端执行器 (黄色球)

    参数:
        viewer:  mujoco.viewer 实例
        q:      [theta1, phi1, theta2, phi2] (rad)
        target_pos: [x, y, z] 目标位置 (m)，或 None
        rotations:  [r1..r8] 8 舵机圈数，或 None

    返回:
        nodes, tip_pos
    """
    nodes = compute_nodes(q)
    tip_pos = nodes[-1]

    viewer.user_scn.ngeom = 0

    # ---- 基座坐标系 (XYZ 轴) ----
    axis_len = 0.15
    origin = np.zeros(3)
    axes = [
        (np.array([axis_len, 0, 0]), np.array([1, 0, 0, 1])),    # X=红
        (np.array([0, axis_len, 0]), np.array([0, 1, 0, 1])),    # Y=绿
        (np.array([0, 0, axis_len]), np.array([0, 0, 1, 1])),    # Z=蓝
    ]
    for end, color in axes:
        if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
            break
        g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_connector(
            g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.003, origin, end)
        g.rgba[:] = color
        viewer.user_scn.ngeom += 1

    # ---- 机器人节点 (蓝色球) ----
    for p in nodes:
        if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
            break
        g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_initGeom(
            g, type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([0.01, 0, 0]), pos=p,
            mat=np.eye(3).flatten(),
            rgba=np.array([0.2, 0.4, 1.0, 1.0])
        )
        viewer.user_scn.ngeom += 1

    # ---- 骨架 (青色胶囊) ----
    for i in range(len(nodes) - 1):
        if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
            break
        g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_connector(
            g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.008,
            nodes[i], nodes[i + 1])
        g.rgba[:] = np.array([0.2, 0.7, 0.9, 1.0])
        viewer.user_scn.ngeom += 1

    # ---- 末端执行器 (黄色球) ----
    if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
        g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_initGeom(
            g, type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([0.015, 0, 0]), pos=tip_pos,
            mat=np.eye(3).flatten(),
            rgba=np.array([1.0, 0.8, 0.0, 1.0])
        )
        viewer.user_scn.ngeom += 1

    # ---- 目标点 (红色大球 + 虚线连接) ----
    if target_pos is not None:
        tp = np.array(target_pos)
        # 目标球
        if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
            g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
            mujoco.mjv_initGeom(
                g, type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([0.018, 0, 0]), pos=tp,
                mat=np.eye(3).flatten(),
                rgba=np.array([1.0, 0.1, 0.1, 0.9])
            )
            viewer.user_scn.ngeom += 1
        # 末端到目标的连线
        if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
            g = viewer.user_scn.geoms[viewer.user_scn.ngeom]
            mujoco.mjv_connector(
                g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.002,
                tip_pos, tp)
            g.rgba[:] = np.array([1.0, 0.3, 0.3, 0.6])
            viewer.user_scn.ngeom += 1

    return nodes, tip_pos
