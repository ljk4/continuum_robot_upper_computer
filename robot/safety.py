# robot/safety.py -- 安全模块：限幅、障碍物检测、插值器

import numpy as np

from config import robot_cfg, safety_cfg


def clamp_theta(q):
    """夹紧弯曲角到单节安全范围"""
    t1_max = np.deg2rad(robot_cfg.section1_theta_max_deg)
    t2_max = np.deg2rad(robot_cfg.section2_theta_max_deg)

    q_c = q.copy()
    was_clamped = False

    if abs(q_c[0]) > t1_max:
        q_c[0] = np.clip(q_c[0], -t1_max, t1_max)
        was_clamped = True
    if abs(q_c[2]) > t2_max:
        q_c[2] = np.clip(q_c[2], -t2_max, t2_max)
        was_clamped = True

    return q_c, was_clamped


def check_obstacle(target_pos):
    """检查目标位置是否在障碍物区域内"""
    if not safety_cfg.obstacle_awareness:
        return True, ""

    target = np.array(target_pos)
    for i, zone in enumerate(safety_cfg.obstacle_zones):
        center = np.array(zone["center"])
        radius = zone["radius"]
        dist = np.linalg.norm(target - center)
        if dist < radius:
            return False, (
                f"目标 {target_pos} 在障碍区 {i} 内 "
                f"(中心={zone['center']}, 半径={radius}m)"
            )
    return True, ""


def limit_position_change(new_target, last_target, max_change=None):
    """限制目标位置变化幅度"""
    if max_change is None:
        max_change = safety_cfg.max_position_change

    delta = np.array(new_target) - np.array(last_target)
    dist = np.linalg.norm(delta)
    if dist > max_change:
        delta = delta * (max_change / dist)

    return (np.array(last_target) + delta).tolist()


def check_cable_delta(new_rotations, current_rotations, max_delta=None):
    """检查绳长变化是否超限

    返回:
        (safe, max_delta_seen): safe=True 表示全部在限内
    """
    if max_delta is None:
        max_delta = safety_cfg.max_cable_delta

    deltas = [abs(n - c) for n, c in zip(new_rotations, current_rotations)]
    max_d = max(deltas)
    return max_d <= max_delta, max_d


class RotationInterpolator:
    """绳长目标插值器

    维护当前状态和目标状态，每次 get_next_step() 返回下一步值，
    每根绳每步最多移动 max_cable_delta。
    """

    def __init__(self, max_cable_delta=None):
        self.max_cable_delta = (
            max_cable_delta if max_cable_delta is not None
            else safety_cfg.max_cable_delta
        )
        self.current = [0.0] * 8
        self.target = [0.0] * 8

    @property
    def is_interpolating(self):
        return any(
            abs(c - t) > 1e-8
            for c, t in zip(self.current, self.target)
        )

    def update_target(self, new_target):
        self.target = list(new_target)

    def sync_current(self, encoder_values):
        """将 current 同步到编码器反馈的实际位置。"""
        self.current = list(encoder_values)

    def get_next_step(self):
        next_step = [0.0] * 8
        step = self.max_cable_delta

        for i in range(8):
            delta = self.target[i] - self.current[i]
            if abs(delta) <= step:
                next_step[i] = self.target[i]
            else:
                next_step[i] = self.current[i] + (
                    step if delta > 0 else -step
                )

        self.current = next_step[:]
        return next_step

    def reset(self, rotations):
        self.current = list(rotations)
        self.target = list(rotations)
