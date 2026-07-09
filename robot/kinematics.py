# robot/kinematics.py -- PCC 两节连续体机器人运动学

import numpy as np

from config import robot_cfg as cfg, safety_cfg
from robot.safety import clamp_theta


def inverse_kinematics(target, q0=None):
    """逆运动学求解

    输入:
        target -- [x, y, z] 末端目标位置 (m)，机器人坐标系
        q0     -- 上次 IK 解 (可选)，用作初始猜测加速收敛

    输出:
        (rotations, q) — 成功时
        (None, q0)     — 障碍物阻挡 或 目标不可达(IK不收敛)
        rotations -- 8 根舵机目标圈数 (revolutions)
        q         -- [theta1, phi1, theta2, phi2] (rad)，每节弯曲角
    """
    target_arr = np.array(target, dtype=np.float64)

    q, tendons = _robot.task_to_tendon(target_arr, q0)

    # 验证 IK 收敛性：FK(q) 应接近 target
    fk_pos = _robot.tip_position(q)
    fk_error = np.linalg.norm(target_arr - fk_pos)
    if fk_error > cfg.ik_convergence_tol:
        return None, q0

    # 绳位移 (m) → 舵机圈数
    # 注意负号：PCC 文献中 dl>0 表示绳伸长，而舵机正转=收紧(缩短)
    # 因此 rotation = -dl / (π·D)
    spool_circ = np.pi * cfg.spool_diameter
    rotations = -tendons / spool_circ

    return rotations.tolist(), q


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


class PCCSection:
    """单节 PCC 模型"""

    def __init__(self, L, r, num_cables=4):
        self.L = L
        self.r = r
        self.num_cables = num_cables

    def config_to_tendon(self, theta, phi):
        if self.num_cables == 4:
            dl1 = -self.r * theta * np.cos(phi)
            dl2 =  self.r * theta * np.sin(phi)
            dl3 =  self.r * theta * np.cos(phi)
            dl4 = -self.r * theta * np.sin(phi)
            return np.array([dl1, dl2, dl3, dl4])
        elif self.num_cables == 2:
            # 2 绳拮抗对：布置在 0° 和 180° 方位角（对称分布，与绳1、绳3同方位）
            dl1 = -self.r * theta * np.cos(phi)    # 0° 绳
            dl2 =  self.r * theta * np.cos(phi)    # 180° 绳（与 0° 绳反向拮抗）
            return np.array([dl1, dl2])
        else:
            raise ValueError(f"Unsupported cable count: {self.num_cables}")

    def transform(self, theta, phi):
        T = np.eye(4)
        if abs(theta) < 1e-8:
            T[2, 3] = self.L
            return T

        rho = self.L / theta
        px = rho * (1 - np.cos(theta)) * np.cos(phi)
        py = rho * (1 - np.cos(theta)) * np.sin(phi)
        pz = rho * np.sin(theta)

        R = rotz(phi) @ roty(theta) @ rotz(-phi)
        T[:3, :3] = R
        T[:3, 3] = [px, py, pz]
        return T


class MultiSectionRobot:
    """多段连续体机器人运动学模型

    第一部分：n 个相同 PCC 段串联，section1_cables 根绳锚定在第一部分末端
    第二部分：m 个相同 PCC 段串联，section2_cables 根绳穿过第一部分锚定在第二部分末端
    总计 section1_cables + section2_cables 根绳，配置空间 4 维 [theta1, phi1, theta2, phi2]
    """

    def __init__(self):
        self.n = cfg.section1_segments
        self.m = cfg.section2_segments
        self.sec1 = PCCSection(L=cfg.section1_length, r=cfg.section1_radius,
                               num_cables=cfg.section1_cables)
        self.sec2 = PCCSection(L=cfg.section2_length, r=cfg.section2_radius,
                               num_cables=cfg.section2_cables)

    def forward(self, theta1, phi1, theta2, phi2):
        T = np.eye(4)
        T1_single = self.sec1.transform(theta1, phi1)
        for _ in range(self.n):
            T = T @ T1_single
        T2_single = self.sec2.transform(theta2, phi2)
        for _ in range(self.m):
            T = T @ T2_single
        return T

    def tip_position(self, q):
        T = self.forward(q[0], q[1], q[2], q[3])
        return T[:3, 3]

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

    def tendon_jacobian(self, q, h=None):
        """(num_cables)×4 腱映射雅可比矩阵：d(tendon_i)/d(q_j)"""
        if h is None:
            h = cfg.jacobian_step
        f0 = self.config_to_all_tendons(q)
        n = len(f0)
        J = np.zeros((n, 4))
        for i in range(4):
            q2 = q.copy()
            q2[i] += h
            f1 = self.config_to_all_tendons(q2)
            J[:, i] = (f1 - f0) / h
        return J

    def tendon_to_config(self, target_tendons, q0=None,
                         max_iter=None, tol=None):
        """驱动空间 → 配置空间逆映射

        给定 N 个腱位移，用阻尼最小二乘求解最接近的 q。
        用于仿真验证：将插值后的圈数转回 q，通过 FK 得到实际末端位置。

        返回:
            q — 最佳匹配的 [theta1, phi1, theta2, phi2] (rad)
        """
        if max_iter is None:
            max_iter = cfg.ik_max_iter
        if tol is None:
            tol = cfg.ik_tolerance

        if q0 is None:
            q = np.zeros(4)
        else:
            q = q0.copy()

        lam = cfg.ik_damping

        for _ in range(max_iter):
            tendons = self.config_to_all_tendons(q)
            error = target_tendons - tendons
            if np.linalg.norm(error) < tol:
                break

            J = self.tendon_jacobian(q)
            n = J.shape[0]
            JT = J.T
            dq = (
                JT @ np.linalg.solve(J @ JT + lam * np.eye(n), error)
            )

            err_norm = np.linalg.norm(error)
            alpha = 1.0
            accepted = False
            for _ in range(5):
                q_new = q + alpha * dq
                tendons_new = self.config_to_all_tendons(q_new)
                if np.linalg.norm(target_tendons - tendons_new) < err_norm:
                    q = q_new
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                break

        q, _ = clamp_theta(q)
        return q

    def _compute_penalty_gradient(self, q, J_pos):
        """计算三项惩罚函数的 q 空间梯度之和

        返回 dq_penalty (4,)：
        (1) 障碍物排斥势场 — 仅当 obstacle_awareness 开启
        (2) 弯曲角梯度惩罚 — theta1/theta2 接近上限时激活
        (3) 绳长梯度惩罚 — 圈数超过软限制时激活
        """
        dq_penalty = np.zeros(4)

        # (1) 障碍物排斥势场
        if safety_cfg.obstacle_awareness:
            pos = self.tip_position(q)
            for zone in safety_cfg.obstacle_zones:
                center = np.array(zone["center"])
                radius = zone["radius"]
                d_safe = radius * safety_cfg.obstacle_margin
                dist = np.linalg.norm(pos - center)
                if dist < d_safe and dist > 1e-8:
                    dir_away = (pos - center) / dist
                    force_mag = (safety_cfg.obstacle_penalty_weight
                                 * (d_safe - dist) / d_safe)
                    force = force_mag * dir_away
                    dq_penalty += J_pos.T @ force

        # (2) 弯曲角梯度惩罚
        t1_max = np.deg2rad(cfg.section1_theta_max_deg)
        t2_max = np.deg2rad(cfg.section2_theta_max_deg)
        soft1 = t1_max * safety_cfg.theta_soft_ratio
        soft2 = t2_max * safety_cfg.theta_soft_ratio

        for idx, (val, soft, hard_max) in enumerate(
                [(q[0], soft1, t1_max), (q[2], soft2, t2_max)]):
            i = 0 if idx == 0 else 2
            if abs(val) > soft:
                excess = abs(val) - soft
                grad = 2.0 * excess * np.sign(val)
                dq_penalty[i] -= safety_cfg.theta_penalty_weight * grad

        # (3) 绳长梯度惩罚
        tendons = self.config_to_all_tendons(q)
        spool_circ = np.pi * cfg.spool_diameter
        rotations = -tendons / spool_circ
        soft_max = safety_cfg.cable_soft_max
        n = len(tendons)
        grad_rot = np.zeros(n)
        active = False
        for i in range(n):
            if abs(rotations[i]) > soft_max:
                excess = abs(rotations[i]) - soft_max
                grad_rot[i] = 2.0 * excess * np.sign(rotations[i])
                active = True
        if active:
            J_tendon = self.tendon_jacobian(q)
            dq_cable = -(1.0 / spool_circ) * J_tendon.T @ grad_rot
            dq_penalty -= safety_cfg.cable_penalty_weight * dq_cable

        # (4) 最小位移引导 — 始终激活，优选绳位移小的解
        #     作为外部梯度偏置叠加，不放入正规方程 RHS，
        #     避免 A⁻¹ 在 J 零空间方向过度放大
        if safety_cfg.min_displacement_weight > 0:
            J_tendon = self.tendon_jacobian(q)
            dq_penalty -= (safety_cfg.min_displacement_weight
                           * J_tendon.T @ tendons)

        return dq_penalty

    def _compute_penalty_cost(self, q):
        """计算所有软约束惩罚项的标量代价（用于线搜索）

        包含与 _compute_penalty_gradient 一致的三个势场：
        (1) 障碍物排斥势场 P = Σ w/(2·d_safe) · max(0, d_safe - dist)²
        (2) 弯曲角惩罚   P = Σ w · max(0, |θ| - soft)²
        (3) 绳长惩罚     P = Σ w · max(0, |rot| - soft_max)²

        返回:
            cost — 标量代价值 (float)
        """
        cost = 0.0

        # (1) 障碍物排斥势场
        if safety_cfg.obstacle_awareness:
            pos = self.tip_position(q)
            for zone in safety_cfg.obstacle_zones:
                center = np.array(zone["center"])
                radius = zone["radius"]
                d_safe = radius * safety_cfg.obstacle_margin
                dist = np.linalg.norm(pos - center)
                if dist < d_safe:
                    cost += (safety_cfg.obstacle_penalty_weight
                             / (2.0 * d_safe) * (d_safe - dist)**2)

        # (2) 弯曲角惩罚
        t1_max = np.deg2rad(cfg.section1_theta_max_deg)
        t2_max = np.deg2rad(cfg.section2_theta_max_deg)
        soft1 = t1_max * safety_cfg.theta_soft_ratio
        soft2 = t2_max * safety_cfg.theta_soft_ratio

        for val, soft in [(q[0], soft1), (q[2], soft2)]:
            if abs(val) > soft:
                excess = abs(val) - soft
                cost += safety_cfg.theta_penalty_weight * excess**2

        # (3) 绳长惩罚
        tendons = self.config_to_all_tendons(q)
        spool_circ = np.pi * cfg.spool_diameter
        rotations = -tendons / spool_circ
        soft_max = safety_cfg.cable_soft_max
        for rot in rotations:
            if abs(rot) > soft_max:
                excess = abs(rot) - soft_max
                cost += safety_cfg.cable_penalty_weight * excess**2

        return cost

    def inverse_kinematics(self, target, q0=None, max_iter=None, tol=None):
        if max_iter is None:
            max_iter = cfg.ik_max_iter
        if tol is None:
            tol = cfg.ik_tolerance

        if q0 is None:
            guess = cfg.ik_initial_guess_deg
            q = np.array([
                np.deg2rad(guess[0]), np.deg2rad(guess[1]),
                np.deg2rad(guess[2]), np.deg2rad(guess[3])
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
            # 优化框架:
            #   min ||target-f(q+dq)||² + w_disp·||tendons(q+dq)||² + penalty(q+dq) + λ||dq||²
            # 位置项 → 正规方程; 最小位移+软约束 → 外部梯度偏置
            # 注: 最小位移偏置不放入 RHS，避免 A⁻¹ 在 J 零空间方向过度放大
            J_tendon = self.tendon_jacobian(q)
            tendons = self.config_to_all_tendons(q)
            w_disp = safety_cfg.min_displacement_weight
            A = (J.T @ J + w_disp * J_tendon.T @ J_tendon
                 + lam * np.eye(4))
            dq_task = np.linalg.solve(A, J.T @ error)
            dq_penalty = self._compute_penalty_gradient(q, J)
            dq = dq_task + dq_penalty

            # 线搜索: 总代价 = 位置误差² + 最小位移 + 软约束惩罚
            current_cost = (np.sum(error**2)
                            + w_disp * np.sum(tendons**2)
                            + self._compute_penalty_cost(q))
            alpha = 1.0
            accepted = False
            for _ in range(5):
                q_new = q + alpha * dq
                q_new, _ = clamp_theta(q_new)
                pos_new = self.tip_position(q_new)
                error_new = target - pos_new
                tendons_new = self.config_to_all_tendons(q_new)
                new_cost = (np.sum(error_new**2)
                            + w_disp * np.sum(tendons_new**2)
                            + self._compute_penalty_cost(q_new))
                if new_cost < current_cost:
                    q = q_new
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                break

        q, _ = clamp_theta(q)
        return q

    def config_to_all_tendons(self, q):
        dl1 = self.sec1.config_to_tendon(q[0], q[1])
        dl2 = self.sec2.config_to_tendon(q[2], q[3])

        # 第二部分绳穿过第一部分的耦合项：
        # 每根第二节绳取同一方位角的第一节绳通道耦合
        # 如第二段有 2 根绳（0°、180°），则取第一段 0°(idx0) 和 180°(idx2) 的耦合
        coupling = dl1 * self.n
        step = self.sec1.num_cables // self.sec2.num_cables  # 均匀取样的步长

        return np.concatenate([
            dl1 * self.n,
            dl2 * self.m + coupling[::step]
        ])

    def task_to_tendon(self, target, q0=None):
        q = self.inverse_kinematics(target, q0)
        tendons = self.config_to_all_tendons(q)
        return q, tendons


_robot = MultiSectionRobot()


# =====================================================
# 测试 (python -m robot.kinematics)
# =====================================================

if __name__ == "__main__":
    robot = MultiSectionRobot()

    print("\n" + "=" * 60)
    print("FK -> IK -> FK TEST")
    print("=" * 60)

    q_true = np.array([
        np.deg2rad(40), np.deg2rad(30),
        np.deg2rad(25), np.deg2rad(60)
    ])

    print("\n真实配置:")
    print("theta1 =", np.rad2deg(q_true[0]))
    print("phi1 =", np.rad2deg(q_true[1]))
    print("theta2 =", np.rad2deg(q_true[2]))
    print("phi2 =", np.rad2deg(q_true[3]))

    target = robot.tip_position(q_true)
    print("\n目标位置:")
    print(target)

    q_est = robot.inverse_kinematics(target)
    print("\nIK求解结果:")
    print("theta1 =", np.rad2deg(q_est[0]))
    print("phi1 =", np.rad2deg(q_est[1]))
    print("theta2 =", np.rad2deg(q_est[2]))
    print("phi2 =", np.rad2deg(q_est[3]))

    verify = robot.tip_position(q_est)
    print("\nFK验证位置:")
    print(verify)

    error = np.linalg.norm(target - verify)
    print("\n位置误差(m):")
    print(error)

    tendons = robot.config_to_all_tendons(q_est)
    print(f"\n{len(tendons)}根绳位移(mm):")
    print(np.round(tendons * 1000, 3))
