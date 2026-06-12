# vision/tracker.py -- AprilTag 视觉位姿检测与可视化

import cv2
import numpy as np

from config import vision_cfg as cfg


class VisionTracker:
    def __init__(self):
        aruco_enum = getattr(cv2.aruco, cfg.aruco_dict)
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_enum)
        aruco_params = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        self.tag_size = cfg.tag_size
        self.camera_matrix = cfg.camera_matrix
        self.dist_coeffs = cfg.dist_coeffs
        self.cam_to_robot_R = cfg.cam_to_robot_R
        self.cam_to_robot_t = cfg.cam_to_robot_t

        self.cap = cv2.VideoCapture(cfg.camera_index)

        # 应用相机参数配置（不支持的直接跳过）
        def _try_set(prop, value):
            try:
                if value != 0:
                    self.cap.set(prop, value)
            except Exception:
                pass

        _try_set(cv2.CAP_PROP_FRAME_WIDTH, cfg.camera_width)
        _try_set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.camera_height)
        _try_set(cv2.CAP_PROP_FPS, cfg.camera_fps)
        # 曝光：先关自动曝光再设手动值
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 手动曝光
            if cfg.camera_exposure != 0:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, cfg.camera_exposure)
        except Exception:
            pass

        print(f"[vision] Camera: "
              f"{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
              f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ "
              f"{self.cap.get(cv2.CAP_PROP_FPS):.0f}fps")

        self.obj_points = np.array([
            [-self.tag_size / 2,  self.tag_size / 2, 0],
            [ self.tag_size / 2,  self.tag_size / 2, 0],
            [ self.tag_size / 2, -self.tag_size / 2, 0],
            [-self.tag_size / 2, -self.tag_size / 2, 0]
        ], dtype=np.float32)

    def get_pose(self):
        """返回检测结果字典，未检测到时返回 {"frame": frame, "pose": None}"""
        ret, frame = self.cap.read()
        if not ret:
            return None

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return {"frame": frame, "pose": None}

        current_corners = corners[0][0]
        tag_id = int(ids[0][0])

        success, rvec, tvec = cv2.solvePnP(
            self.obj_points, current_corners,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            return {"frame": frame, "pose": None}

        tvec_cam = tvec.flatten()
        tvec_robot = self.cam_to_robot_R @ tvec_cam + self.cam_to_robot_t

        reprojected, _ = cv2.projectPoints(
            self.obj_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        reprojected = reprojected.reshape(-1, 2)
        reproj_error = np.mean(
            np.linalg.norm(current_corners - reprojected, axis=1)
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
    """在图像上绘制检测结果"""
    frame = result["frame"].copy()

    if result["pose"] is None:
        cv2.putText(frame, "No Tag Detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return frame

    rvec = result["rvec"]
    tvec = result["tvec"]
    corners = result["corners"]
    tag_id = result["tag_id"]
    reproj_error = result["reproj_error"]
    x, y, z = result["pose"]

    # 2D 边框
    pts = corners.astype(int)
    for j in range(4):
        cv2.line(frame, tuple(pts[j]), tuple(pts[(j + 1) % 4]), (0, 255, 0), 2)
    for pt in pts:
        cv2.circle(frame, tuple(pt), 4, (0, 255, 0), -1)

    # 3D 坐标轴
    cv2.drawFrameAxes(frame, cfg.camera_matrix, cfg.dist_coeffs,
                      rvec, tvec, cfg.tag_size * 0.8)

    # 信息文字
    cv2.putText(frame, f"ID: {tag_id}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f"X: {x:.4f} m", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(frame, f"Y: {y:.4f} m", (20, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"Z: {z:.4f} m", (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    err_color = (0, 255, 0) if reproj_error < 2.0 else (0, 0, 255)
    cv2.putText(frame, f"Reproj Error: {reproj_error:.2f} px", (20, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, err_color, 2)

    # 重投影点
    repr_pts, _ = cv2.projectPoints(
        np.array([
            [-cfg.tag_size / 2,  cfg.tag_size / 2, 0],
            [ cfg.tag_size / 2,  cfg.tag_size / 2, 0],
            [ cfg.tag_size / 2, -cfg.tag_size / 2, 0],
            [-cfg.tag_size / 2, -cfg.tag_size / 2, 0]
        ], dtype=np.float32),
        rvec, tvec, cfg.camera_matrix, cfg.dist_coeffs
    )
    repr_pts = repr_pts.reshape(-1, 2).astype(int)
    for pt in repr_pts:
        cv2.circle(frame, tuple(pt), 3, (0, 255, 255), -1)

    return frame
