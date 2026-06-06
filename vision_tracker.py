# vision_tracker.py -- AprilTag 视觉位姿检测与可视化

import cv2
import numpy as np

from config import vision_cfg as cfg


class VisionTracker:

    def __init__(self):

        aruco_enum = getattr(
            cv2.aruco,
            cfg.aruco_dict
        )

        aruco_dict = (
            cv2.aruco
            .getPredefinedDictionary(aruco_enum)
        )

        aruco_params = (
            cv2.aruco.DetectorParameters()
        )

        self.detector = cv2.aruco.ArucoDetector(
            aruco_dict,
            aruco_params
        )

        self.tag_size = cfg.tag_size

        self.camera_matrix = cfg.camera_matrix

        self.dist_coeffs = cfg.dist_coeffs

        self.cam_to_robot_R = cfg.cam_to_robot_R
        self.cam_to_robot_t = cfg.cam_to_robot_t

        self.cap = cv2.VideoCapture(
            cfg.camera_index
        )

        self.obj_points = np.array([
            [-self.tag_size / 2,
              self.tag_size / 2, 0],
            [ self.tag_size / 2,
              self.tag_size / 2, 0],
            [ self.tag_size / 2,
             -self.tag_size / 2, 0],
            [-self.tag_size / 2,
             -self.tag_size / 2, 0]
        ], dtype=np.float32)

    def get_pose(self):
        """
        返回:
            dict: 包含检测结果的字典，未检测到时返回 None
                frame:          原始图像帧
                pose:           [x, y, z] 位置 (m)
                rvec:           旋转向量 (3x1)
                tvec:           平移向量 (3x1)
                corners:        检测到的角点
                tag_id:         标记 ID
                reproj_error:   重投影误差 (像素)
        """

        ret, frame = self.cap.read()

        if not ret:
            return None

        corners, ids, _ = (
            self.detector.detectMarkers(frame)
        )

        if ids is None:
            return {"frame": frame, "pose": None}

        # 取第一个检测到的标记
        current_corners = corners[0][0]
        tag_id = int(ids[0][0])

        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            current_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            return {"frame": frame, "pose": None}

        # 相机坐标系 → 机器人坐标系
        tvec_cam = tvec.flatten()
        tvec_robot = (
            self.cam_to_robot_R @ tvec_cam
            + self.cam_to_robot_t
        )

        # 计算重投影误差
        reprojected, _ = cv2.projectPoints(
            self.obj_points,
            rvec, tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        reprojected = reprojected.reshape(-1, 2)

        reproj_error = np.mean(
            np.linalg.norm(
                current_corners - reprojected,
                axis=1
            )
        )

        return {
            "frame": frame,
            "pose": tvec_robot.tolist(),
            "rvec": rvec,
            "tvec": tvec,
            "corners": current_corners,
            "tag_id": tag_id,
            "reproj_error": reproj_error,
        }

    def release(self):

        self.cap.release()


def draw_detection(result):
    """在图像上绘制检测结果

    参数:
        result: get_pose() 返回的字典

    返回:
        绘制了标注的图像 (BGR)
    """

    frame = result["frame"].copy()

    if result["pose"] is None:
        # 未检测到：显示提示文字
        cv2.putText(
            frame,
            "No Tag Detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 255), 2
        )
        return frame

    rvec = result["rvec"]
    tvec = result["tvec"]
    corners = result["corners"]
    tag_id = result["tag_id"]
    reproj_error = result["reproj_error"]
    x, y, z = result["pose"]

    # ---- 1. 绘制检测到的 2D 边框 ----
    pts = corners.astype(int)
    for j in range(4):
        cv2.line(
            frame,
            tuple(pts[j]),
            tuple(pts[(j + 1) % 4]),
            (0, 255, 0), 2
        )

    # 在角点处画圆
    for pt in pts:
        cv2.circle(frame, tuple(pt), 4, (0, 255, 0), -1)

    # ---- 2. 绘制 3D 坐标轴 ----
    # 红=X, 绿=Y, 蓝=Z，轴长 = tag_size
    cv2.drawFrameAxes(
        frame,
        cfg.camera_matrix,
        cfg.dist_coeffs,
        rvec, tvec,
        cfg.tag_size * 0.8
    )

    # ---- 3. 绘制信息文字 ----
    # 标签 ID
    cv2.putText(
        frame,
        f"ID: {tag_id}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7, (0, 255, 255), 2
    )

    # 位置坐标
    cv2.putText(
        frame,
        f"X: {x:.4f} m",
        (20, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (0, 0, 255), 2    # 红色 = X
    )
    cv2.putText(
        frame,
        f"Y: {y:.4f} m",
        (20, 85),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (0, 255, 0), 2    # 绿色 = Y
    )
    cv2.putText(
        frame,
        f"Z: {z:.4f} m",
        (20, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (255, 0, 0), 2    # 蓝色 = Z
    )

    # 重投影误差
    err_color = (0, 255, 0) if reproj_error < 2.0 else (0, 0, 255)
    cv2.putText(
        frame,
        f"Reproj Error: {reproj_error:.2f} px",
        (20, 140),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6, err_color, 2
    )

    # 重投影点（黄色小点，验证精度）
    repr_pts, _ = cv2.projectPoints(
        np.array([
            [-cfg.tag_size / 2,  cfg.tag_size / 2, 0],
            [ cfg.tag_size / 2,  cfg.tag_size / 2, 0],
            [ cfg.tag_size / 2, -cfg.tag_size / 2, 0],
            [-cfg.tag_size / 2, -cfg.tag_size / 2, 0]
        ], dtype=np.float32),
        rvec, tvec,
        cfg.camera_matrix, cfg.dist_coeffs
    )
    repr_pts = repr_pts.reshape(-1, 2).astype(int)
    for pt in repr_pts:
        cv2.circle(frame, tuple(pt), 3, (0, 255, 255), -1)

    return frame


# =====================================================
# 兼容接口
# =====================================================

_tracker = None


def vision_get_pose():
    """
    返回:
        [x, y, z] 末端位置 (m)
        未检测到时返回 None
    """

    global _tracker

    if _tracker is None:
        _tracker = VisionTracker()

    result = _tracker.get_pose()

    if result is None or result.get("pose") is None:
        return None

    return result["pose"]


def vision_get_result():
    """
    返回:
        完整检测结果字典（含 frame, pose, rvec, tvec 等）
        未检测到时返回 {"frame": frame, "pose": None}
    """

    global _tracker

    if _tracker is None:
        _tracker = VisionTracker()

    return _tracker.get_pose()
