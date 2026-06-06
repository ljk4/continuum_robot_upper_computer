# 逆运动学

import numpy as np

from config import robot_cfg as cfg


def inverse_kinematics(target, q0=None):
    """
    输入:
        target -- [x, y, z] 末端目标位置 (m)，机器人坐标系
        q0     -- 上次 IK 解 (可选)，用作初始猜测加速收敛

    输出:
        (rotations, q)
        rotations -- 8 根舵机目标圈数 (revolutions)
        q         -- [theta1, phi1, theta2, phi2] (rad)，每节弯曲角

    注意: theta1/theta2 是单节弯曲角，总弯曲角 = 段数 × theta
    """

    q, tendons = _robot.task_to_tendon(
        np.array(target, dtype=np.float64), q0
    )

    # 绳位移 (m) -> 舵机圈数
    # 周长 = pi * 直径，圈数 = 位移 / 周长
    spool_circ = np.pi * cfg.spool_diameter
    rotations = tendons / spool_circ

    return rotations.tolist(), q


# =====================================================
# 基础旋转矩阵
# =====================================================

def rotx(a):

    c = np.cos(a)
    s = np.sin(a)

    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def roty(a):

    c = np.cos(a)
    s = np.sin(a)

    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def rotz(a):

    c = np.cos(a)
    s = np.sin(a)

    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])


# =====================================================
# 单节连续体
# =====================================================

class PCCSection:

    def __init__(self, L, r):
        """单节 PCC 模型

        参数:
            L: 单节段长 (m)
            r: 绳分布半径 (m)
        """
        self.L = L
        self.r = r

    # ----------------------------------------
    # 配置空间 -> 驱动空间
    # ----------------------------------------
    def config_to_tendon(self, theta, phi):

        dl1 = -self.r * theta * np.cos(phi)

        dl2 = self.r * theta * np.sin(phi)

        dl3 = self.r * theta * np.cos(phi)

        dl4 = -self.r * theta * np.sin(phi)

        return np.array([
            dl1,
            dl2,
            dl3,
            dl4
        ])

    # ----------------------------------------
    # 单节变换矩阵
    # ----------------------------------------
    def transform(self, theta, phi):

        T = np.eye(4)

        if abs(theta) < 1e-8:

            T[2, 3] = self.L
            return T

        rho = self.L / theta

        px = rho * (1 - np.cos(theta)) * np.cos(phi)
        py = rho * (1 - np.cos(theta)) * np.sin(phi)
        pz = rho * np.sin(theta)

        R = (
            rotz(phi)
            @ roty(theta)
            @ rotz(-phi)
        )

        T[:3, :3] = R
        T[:3, 3] = [px, py, pz]

        return T

    def sample_arc_by_transform(section,
                            theta,
                            phi,
                            num_points=20):

        pts = []

        for ratio in np.linspace(
                0.0,
                1.0,
                num_points):

            T = section.transform(
                theta * ratio,
                phi
            )

            pts.append(
                T[:3,3].copy()
            )

        return np.array(pts)

# =====================================================
# 多段连续体机器人
# =====================================================

class MultiSectionRobot:
    """多段连续体机器人运动学模型

    结构：
        第一部分：n 个相同 PCC 段串联，4 根绳锚定在第一部分末端
        第二部分：m 个相同 PCC 段串联，4 根绳穿过第一部分锚定在第二部分末端
        总计 8 根绳，配置空间 4 维 [theta1, phi1, theta2, phi2]
    """

    def __init__(self):
        # 第一部分段数 n，第二部分段数 m
        self.n = cfg.section1_segments
        self.m = cfg.section2_segments
        # 两部分各自独立的单节模型（段长和绳分布半径不同）
        self.sec1 = PCCSection(
            L=cfg.section1_length,
            r=cfg.section1_radius
        )
        self.sec2 = PCCSection(
            L=cfg.section2_length,
            r=cfg.section2_radius
        )

    # ----------------------------------------
    # 正运动学
    # ----------------------------------------
    def forward(self, theta1, phi1, theta2, phi2):
        """正运动学：T_total = T1^n @ T2^m

        参数（均为**单节**弯曲角，非总弯曲角）：
            theta1: 第一部分每节弯曲角 (rad)，总弯曲角 = n * theta1
            phi1:   第一部分弯曲方向 (rad)
            theta2: 第二部分每节弯曲角 (rad)，总弯曲角 = m * theta2
            phi2:   第二部分弯曲方向 (rad)
        """

        T = np.eye(4)

        # 第一部分：n 段级联（使用 sec1 的几何参数）
        T1_single = self.sec1.transform(theta1, phi1)
        for _ in range(self.n):
            T = T @ T1_single

        # 第二部分：m 段级联（使用 sec2 的几何参数）
        T2_single = self.sec2.transform(theta2, phi2)
        for _ in range(self.m):
            T = T @ T2_single

        return T

    # ----------------------------------------
    # 配置空间 q -> 末端位置
    # ----------------------------------------
    def tip_position(self, q):

        T = self.forward(
            q[0], q[1], q[2], q[3]
        )

        return T[:3, 3]

    # ----------------------------------------
    # 数值 Jacobian (3x4)
    # ----------------------------------------
    def jacobian(self, q, h=None):

        if h is None:
            h = cfg.jacobian_step

        J = np.zeros((3, 4))

        f0 = self.tip_position(q)

        for i in range(4):

            q2 = q.copy()
            q2[i] += h
            f1 = self.tip_position(q2)
            J[:, i] = (f1 - f0) / h

        return J

    # ----------------------------------------
    # 阻尼最小二乘 IK
    # ----------------------------------------
    def inverse_kinematics(
            self, target, q0=None,
            max_iter=None, tol=None):

        if max_iter is None:
            max_iter = cfg.ik_max_iter
        if tol is None:
            tol = cfg.ik_tolerance

        if q0 is None:
            guess = cfg.ik_initial_guess_deg
            q = np.array([
                np.deg2rad(guess[0]),
                np.deg2rad(guess[1]),
                np.deg2rad(guess[2]),
                np.deg2rad(guess[3])
            ])
        else:
            q = q0.copy()

        lam = cfg.ik_damping

        for _ in range(max_iter):

            pos = self.tip_position(q)
            error = target - pos

            if np.linalg.norm(error) < tol:
                break

            J = self.jacobian(q)
            JT = J.T

            dq = (
                JT
                @ np.linalg.inv(
                    J @ JT + lam * np.eye(3)
                )
                @ error
            )

            # 线搜索：全步长优先，误差增大时回溯减半
            err_norm = np.linalg.norm(error)
            alpha = 1.0
            accepted = False
            for _ in range(5):
                q_new = q + alpha * dq
                pos_new = self.tip_position(q_new)
                if np.linalg.norm(target - pos_new) < err_norm:
                    q = q_new
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                break  # 所有步长均未减小误差，终止迭代

        return q

    # ----------------------------------------
    # 配置空间 -> 8 绳位移
    # ----------------------------------------
    def config_to_all_tendons(self, q):
        """计算 8 根绳的总位移

        第一部分 4 根绳（绳1-4）：锚定在第一部分末端
            总位移 = 单节位移 × n

        第二部分 4 根绳（绳5-8）：穿过第一部分，锚定在第二部分末端
            总位移 = 第二部分自身弯曲(m段) + 第一部分弯曲引起的路径变化(n段)
            路径变化符号与第一部分绳位移相反（外侧伸长、内侧缩短）
        """

        dl1 = self.sec1.config_to_tendon(q[0], q[1])
        dl2 = self.sec2.config_to_tendon(q[2], q[3])

        # 第二部分绳穿过第一部分的耦合项：
        # 绳5-8 与绳1-4 通道对应、方位角相同，穿过同一弯曲路径
        # 路径变化符号与第一部分绳位移相同（绳1缩短则绳5同样缩短）
        coupling = dl1 * self.n

        return np.concatenate([dl1 * self.n, dl2 * self.m + coupling])

    # ----------------------------------------
    # 任务空间 -> 驱动空间
    # ----------------------------------------
    def task_to_tendon(self, target, q0=None):

        q = self.inverse_kinematics(target, q0)
        tendons = self.config_to_all_tendons(q)

        return q, tendons


# 模块级机器人实例（供 inverse_kinematics 函数使用）
_robot = MultiSectionRobot()


# =====================================================
# 测试
# =====================================================

if __name__ == "__main__":

    robot = MultiSectionRobot()

    print("\n")
    print("=" * 60)
    print("FK -> IK -> FK TEST")
    print("=" * 60)

    # ----------------------------------
    # 真值配置
    # ----------------------------------

    q_true = np.array([

        np.deg2rad(40),
        np.deg2rad(30),

        np.deg2rad(25),
        np.deg2rad(60)

    ])

    print("\n真实配置:")

    print(
        "theta1 =",
        np.rad2deg(q_true[0])
    )

    print(
        "phi1 =",
        np.rad2deg(q_true[1])
    )

    print(
        "theta2 =",
        np.rad2deg(q_true[2])
    )

    print(
        "phi2 =",
        np.rad2deg(q_true[3])
    )

    # ----------------------------------
    # FK
    # ----------------------------------

    target = robot.tip_position(
        q_true
    )

    print("\n目标位置:")

    print(target)

    # ----------------------------------
    # IK
    # ----------------------------------

    q_est = robot.inverse_kinematics(
        target
    )
 
    print("\nIK求解结果:")

    print(
        "theta1 =",
        np.rad2deg(q_est[0])
    )

    print(
        "phi1 =",
        np.rad2deg(q_est[1])
    )

    print(
        "theta2 =",
        np.rad2deg(q_est[2])
    )

    print(
        "phi2 =",
        np.rad2deg(q_est[3])
    )

    # ----------------------------------
    # FK验证
    # ----------------------------------

    verify = robot.tip_position(
        q_est
    )

    print("\nFK验证位置:")

    print(verify)

    error = np.linalg.norm(
        target - verify
    )

    print("\n位置误差(m):")

    print(error)

    # ----------------------------------
    # 驱动空间
    # ----------------------------------

    tendons = robot.config_to_all_tendons(
        q_est
    )

    print("\n8根绳位移(mm):")

    print(
        np.round(
            tendons * 1000,
            3
        )
    )