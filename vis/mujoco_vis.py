# vis/mujoco_vis.py -- MuJoCo 3D 可视化线程

import time
import threading

import numpy as np
import mujoco
import mujoco.viewer

from vis.mujoco_model import compute_nodes, draw_scene, XML
from config import robot_cfg
from utils.logger import setup_logger

log = setup_logger("mujoco_vis")


class MuJoCoVisThread(threading.Thread):
    """MuJoCo 3D 可视化线程

    从主循环接收状态 (q, rotations, target_pos)，
    实时渲染机器人骨架、目标点、坐标系，并输出三空间参数。
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.q = np.zeros(4)
        self.rotations = np.zeros(robot_cfg.num_cables)
        self.target_pos = None
        self.lock = threading.Lock()
        self.running = True

    def update_state(self, q, rotations=None, target_pos=None):
        """更新显示状态（主循环调用）"""
        with self.lock:
            self.q = np.array(q, dtype=np.float64)
            if rotations is not None:
                self.rotations = np.array(rotations, dtype=np.float64)
            self.target_pos = target_pos

    def run(self):
        import os as _os
        import sys as _sys
        if _sys.platform != "win32" and \
           "DISPLAY" not in _os.environ and \
           "WAYLAND_DISPLAY" not in _os.environ:
            log.warning("无图形显示服务, MuJoCo 可视化已禁用")
            return

        model = mujoco.MjModel.from_xml_string(XML)
        data = mujoco.MjData(model)

        log.info("MuJoCo 可视化窗口已启动")

        frame_count = 0
        log_interval = 60  # ~1 秒输出一次状态

        try:
            with mujoco.viewer.launch_passive(
                    model, data,
                    show_left_ui=False,
                    show_right_ui=False,
            ) as viewer:
                while viewer.is_running() and self.running:
                    with self.lock:
                        q = self.q.copy()
                        rots = self.rotations.copy()
                        target = self.target_pos

                    mujoco.mj_forward(model, data)
                    draw_scene(viewer, q, target_pos=target, rotations=rots)

                    # 定期输出三空间参数
                    frame_count += 1
                    if frame_count >= log_interval:
                        frame_count = 0
                        tip = compute_nodes(q)[-1]
                        log.info(
                            "[驱动] 圈数: %s",
                            " ".join(f"{r:+.3f}" for r in rots))
                        log.info(
                            "[配置] θ1=%5.1f° φ1=%6.1f°  θ2=%5.1f° φ2=%6.1f°",
                            np.rad2deg(q[0]), np.rad2deg(q[1]),
                            np.rad2deg(q[2]), np.rad2deg(q[3]))
                        log.info(
                            "[任务] 末端: X=%+.4f Y=%+.4f Z=%+.4f",
                            tip[0], tip[1], tip[2])
                        if target is not None:
                            err = np.linalg.norm(target - tip)
                            log.info(
                                "[任务] 目标: X=%+.4f Y=%+.4f Z=%+.4f  |"
                                "|误差|=%.4f",
                                target[0], target[1], target[2], err)

                    viewer.sync()
                    time.sleep(0.01)

        except Exception as e:
            log.error("MuJoCo 可视化异常: %s", e)

        log.info("MuJoCo 可视化窗口已关闭")

    def stop(self):
        self.running = False
