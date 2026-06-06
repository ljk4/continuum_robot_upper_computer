# input/sources.py -- 输入源抽象：视觉 / 手动 / 轨迹

import time
import threading
from abc import ABC, abstractmethod

import numpy as np

from config import input_cfg
from vision.tracker import VisionTracker
from utils.logger import setup_logger

log = setup_logger("input_source")


class InputSource(ABC):
    """输入源抽象接口"""

    @abstractmethod
    def get_target(self):
        """返回 [x, y, z] 目标位置，或 None（无可用目标）"""
        pass

    def get_direct_rotations(self):
        """返回 (rotations, q, position) 或 None

        None 表示走正常 target → IK 流程。
        非 None 时跳过 IK 和位置限幅，直接进入 EMA。
        - rotations: [r1..r8] 8 个舵机圈数
        - q: [θ1,φ1,θ2,φ2] rad 或 None
        - position: [x,y,z] 用于日志显示 或 None
        """
        return None

    def start(self):
        pass

    def stop(self):
        pass


class VisionInput(InputSource):
    """摄像头 AprilTag 视觉输入"""

    def __init__(self):
        self.vision_thread = _VisionThread()

    def start(self):
        self.vision_thread.start()

    def get_target(self):
        return self.vision_thread.get_pose()

    def get_result(self):
        return self.vision_thread.get_result()

    def stop(self):
        self.vision_thread.stop()


class _VisionThread(threading.Thread):
    """后台线程持续采集摄像头帧"""

    def __init__(self):
        super().__init__(daemon=True)
        self.tracker = None
        self.latest_pose = None
        self.latest_result = None
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        try:
            self.tracker = VisionTracker()
            log.info("视觉线程: 摄像头已打开")
        except Exception as e:
            log.error("视觉线程: 摄像头打开失败: %s", e)
            return

        while self.running:
            result = self.tracker.get_pose()
            if result is None:
                time.sleep(0.001)
                continue

            pose = result.get("pose")
            with self.lock:
                self.latest_pose = pose
                self.latest_result = result

    def get_pose(self):
        with self.lock:
            return self.latest_pose

    def get_result(self):
        with self.lock:
            return self.latest_result

    def stop(self):
        self.running = False
        if self.tracker:
            try:
                self.tracker.release()
            except Exception:
                pass


class ManualInput(InputSource):
    """手动输入 — 四种子模式

    - end_effector: 末端位置 [x,y,z] → 走正常 IK
    - rotations:    8 舵机圈数 → 跳过 IK
    - cable_length: 8 绳位移 (m) → 转为圈数，跳过 IK
    - curvature:    [θ1,φ1,θ2,φ2] (度) → FK+tendons，跳过 IK
    """

    def __init__(self, cfg=None):
        if cfg is None:
            cfg = input_cfg
        self.submode = cfg.manual_submode

        if self.submode == "end_effector":
            self._target = list(cfg.manual_end_effector)
            self._direct = None

        elif self.submode == "rotations":
            self._target = None
            self._direct = (list(cfg.manual_rotations), None, None)

        elif self.submode == "cable_length":
            from config import robot_cfg
            spool_circ = np.pi * robot_cfg.spool_diameter
            lengths = cfg.manual_cable_length
            rots = [-l / spool_circ for l in lengths]
            self._target = None
            self._direct = (rots, None, None)

        elif self.submode == "curvature":
            from config import robot_cfg
            from robot.kinematics import MultiSectionRobot
            q_deg = cfg.manual_curvature
            q = np.deg2rad(q_deg)
            robot = MultiSectionRobot()
            pos = robot.tip_position(q).tolist()
            tendons = robot.config_to_all_tendons(q)
            spool_circ = np.pi * robot_cfg.spool_diameter
            rots = (-tendons / spool_circ).tolist()
            self._target = pos
            self._direct = (rots, q.tolist(), pos)

        else:
            raise ValueError(f"未知手动子模式: {self.submode}")

    def get_target(self):
        if self._target is not None:
            return self._target[:]
        return None

    def get_direct_rotations(self):
        return self._direct


class TrajectoryInput(InputSource):
    """按时间生成轨迹目标"""

    def __init__(self, trajectory_type=None, center=None,
                 amplitude=None, frequency=None, axis=None):
        self.trajectory_type = trajectory_type or input_cfg.trajectory_type
        self.center = np.array(center or input_cfg.trajectory_center)
        self.amplitude = amplitude or input_cfg.trajectory_amplitude
        self.frequency = frequency or input_cfg.trajectory_frequency
        self.axis = axis or input_cfg.trajectory_axis
        self.t0 = time.time()

    def get_target(self):
        t = time.time() - self.t0
        offset = np.zeros(3)

        if self.trajectory_type == "sine":
            idx = {"x": 0, "y": 1, "z": 2}[self.axis]
            offset[idx] = self.amplitude * np.sin(
                2 * np.pi * self.frequency * t)

        elif self.trajectory_type == "circle":
            angle = 2 * np.pi * self.frequency * t
            offset[0] = self.amplitude * np.cos(angle)
            offset[1] = self.amplitude * np.sin(angle)

        elif self.trajectory_type == "line":
            idx = {"x": 0, "y": 1, "z": 2}[self.axis]
            period = 1.0 / self.frequency
            phase = (t % period) / period
            if phase < 0.5:
                offset[idx] = self.amplitude * (4 * phase - 1)
            else:
                offset[idx] = self.amplitude * (3 - 4 * phase)

        return (self.center + offset).tolist()


def create_input_source(cfg=None):
    """根据配置创建输入源"""
    if cfg is None:
        cfg = input_cfg

    mode = cfg.input_mode
    if mode == "vision":
        return VisionInput()
    elif mode == "manual":
        return ManualInput()
    elif mode == "trajectory":
        return TrajectoryInput()
    else:
        raise ValueError(f"未知输入模式: {mode}")
