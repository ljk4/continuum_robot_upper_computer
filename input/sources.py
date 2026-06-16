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


class SearchInput(InputSource):
    """眼在手上纯图像伺服搜索模式

    - Tag 检测到: 像素偏差 → 弯曲角 → 圈数（绕过 IK）
    - Tag 丢失: 慢速扫描寻找
    - 不需要标定 cam_to_robot / ee_to_cam
    """

    def __init__(self):
        self._tracker = None
        self._robot = None
        self._spool_circ = None
        self._cx = 640.0
        self._cy = 360.0
        self._Kp = 0.03             # 像素偏差 → theta 增益 (rad/pixel)
        self._Ka = 0.15             # 面积偏差 → theta2 增益
        self._target_area = 8000    # 目标 tag 面积 (像素²)
        self._scan_theta = 0.04     # 扫描弯曲幅度 (rad, ~2.3°)
        self._scan_speed = 0.3      # 扫描频率 (Hz)
        self._deadband_px = 40      # 像素死区 (像素)
        self._deadband_area = 0.15  # 面积死区 (比例)
        self._hold_frames = 15      # 死区内持续 N 帧后锁定 (约 1.5s)
        self._t0 = time.time()
        self._lock = threading.Lock()
        self._latest_result = None
        self._hold_counter = 0
        self._last_rots = None      # 锁定时保持的最后圈数

    def start(self):
        from config import vision_cfg, robot_cfg
        from robot.kinematics import MultiSectionRobot
        self._cx = float(vision_cfg.camera_matrix[0, 2])
        self._cy = float(vision_cfg.camera_matrix[1, 2])
        self._robot = MultiSectionRobot()
        self._spool_circ = np.pi * robot_cfg.spool_diameter
        self._tracker = VisionTracker()
        log.info("Search 模式: 摄像头已打开, 图像中心=(%.0f, %.0f)", self._cx, self._cy)

    def stop(self):
        if self._tracker:
            self._tracker.release()

    def get_target(self):
        return None  # 始终走 direct_rotations 路径

    def get_direct_rotations(self):
        if self._tracker is None:
            return None
        result = self._tracker.get_pose()
        with self._lock:
            self._latest_result = result
        if result is not None and result.get("pose") is not None:
            return self._servo_to_tag(result)
        # Tag 丢失 → 重置锁定状态 → 扫描
        self._hold_counter = 0
        return self._scan_pattern()

    def get_result(self):
        """供 VisThread 调用，获取最新检测帧用于 OpenCV 显示"""
        with self._lock:
            return self._latest_result

    def _servo_to_tag(self, result):
        import cv2
        corners = result.get("corners")
        cx_px = np.mean(corners[:, 0])
        cy_px = np.mean(corners[:, 1])
        area = float(cv2.contourArea(corners.astype(np.float32)))

        ex = cx_px - self._cx
        ey = cy_px - self._cy
        area_err = abs(area - self._target_area) / self._target_area
        pixel_err = np.sqrt(ex**2 + ey**2)

        # 死区 + 滞回: 偏差小则累加计数器，超阈值则重置
        if pixel_err < self._deadband_px and area_err < self._deadband_area:
            self._hold_counter += 1
            if self._hold_counter >= self._hold_frames:
                # 已锁定: 不输出新目标, 插值器保持当前位置
                return None
        else:
            self._hold_counter = 0

        # 像素偏差 → 弯曲
        theta1 = self._Kp * pixel_err / self._cx
        phi1 = np.arctan2(ey, ex)

        # 面积偏差 → 距离调节
        area_err_signed = (area - self._target_area) / self._target_area
        theta2 = -self._Ka * area_err_signed

        from config import robot_cfg
        t1_max = np.deg2rad(robot_cfg.section1_theta_max_deg)
        t2_max = np.deg2rad(robot_cfg.section2_theta_max_deg)
        theta1 = np.clip(theta1, 0.0, t1_max)
        theta2 = np.clip(theta2, -t2_max, t2_max)

        q = np.array([theta1, phi1, theta2, 0.0])
        tendons = self._robot.config_to_all_tendons(q)
        rots = (-tendons / self._spool_circ).tolist()
        self._last_rots = rots
        pos = self._robot.tip_position(q).tolist()
        return (rots, q.tolist(), pos)

    def _scan_pattern(self):
        t = time.time() - self._t0
        phi1 = 2.0 * np.pi * self._scan_speed * t
        from config import robot_cfg
        t1_max = np.deg2rad(robot_cfg.section1_theta_max_deg)
        theta1 = np.clip(self._scan_theta, 0.0, t1_max)
        q = np.array([theta1, phi1, 0.0, 0.0])
        tendons = self._robot.config_to_all_tendons(q)
        rots = (-tendons / self._spool_circ).tolist()
        pos = self._robot.tip_position(q).tolist()
        return (rots, q.tolist(), pos)


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
