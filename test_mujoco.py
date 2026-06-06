import mujoco
import mujoco.viewer

import numpy as np

from ik_solver import MultiSectionRobot


robot = MultiSectionRobot()


# =====================================
# 计算所有PCC节点
# =====================================

def compute_nodes(q):

    theta1, phi1, theta2, phi2 = q

    T = np.eye(4)

    nodes = [np.zeros(3)]

    T1 = robot.sec1.transform(
        theta1,
        phi1
    )

    for _ in range(robot.n):

        T = T @ T1

        nodes.append(
            T[:3, 3].copy()
        )

    T2 = robot.sec2.transform(
        theta2,
        phi2
    )

    for _ in range(robot.m):

        T = T @ T2

        nodes.append(
            T[:3, 3].copy()
        )

    return np.array(nodes)


# =====================================
# XML
# =====================================

XML = """
<mujoco model="continuum">

    <option gravity="0 0 0"/>

    <visual>
        <headlight diffuse="1 1 1"/>
    </visual>

    <worldbody>

        <geom
            type="plane"
            size="2 2 0.1"
            rgba="0.9 0.9 0.9 1"/>

        <body name="base">

            <geom
                type="cylinder"
                size="0.04 0.02"
                rgba="0.4 0.4 0.4 1"/>

        </body>

    </worldbody>

</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)

data = mujoco.MjData(model)


# =====================================
# 启动Viewer
# =====================================

with mujoco.viewer.launch_passive(
        model,
        data) as viewer:

    t = 0.0

    while viewer.is_running():

        # -------------------------
        # 动态测试运动
        # -------------------------

        # target = np.array([
        #     0.4,
        #     0.2,
        #     0.8
        # ])

        # q = robot.inverse_kinematics(
        #     target
        # )

        q = np.deg2rad([
            20,   0,
            25, 180
        ])
        nodes = compute_nodes(q)

        mujoco.mj_forward(
            model,
            data
        )

        # --------------------------------
        # 清空用户几何体
        # --------------------------------

        viewer.user_scn.ngeom = 0

        # --------------------------------
        # 画节点
        # --------------------------------

        for p in nodes:

            if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                break

            g = viewer.user_scn.geoms[
                viewer.user_scn.ngeom
            ]

            mujoco.mjv_initGeom(
                g,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([
                    0.01,
                    0,
                    0
                ]),
                pos=p,
                mat=np.eye(3).flatten(),
                rgba=np.array([
                    0,
                    0,
                    1,
                    1
                ])
            )

            viewer.user_scn.ngeom += 1

        # --------------------------------
        # 画连续体骨架
        # --------------------------------

        for i in range(len(nodes)-1):

            p1 = nodes[i]
            p2 = nodes[i+1]

            if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                break

            g = viewer.user_scn.geoms[
                viewer.user_scn.ngeom
            ]

            mujoco.mjv_connector(
                g,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                0.008,
                p1,
                p2
            )

            g.rgba[:] = np.array([
                0.2,
                0.7,
                0.9,
                1.0
            ])

            viewer.user_scn.ngeom += 1

        viewer.sync()

        t += 0.01