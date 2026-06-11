# gui/shared_state.py — 线程安全状态桥（GUI 线程 ↔ 主控制循环）

import threading


class SharedState:
    """GUI 线程与主控制循环之间的线程安全数据交换中心。

    所有读写方法均用 threading.Lock 保护。
    GUI 写入 → 主循环 consume（一次性消费，避免重复处理）。
    主循环写入 → GUI 定时 poll 读取。
    """

    def __init__(self):
        self._lock = threading.Lock()

        # ── GUI → 主循环 (写入 + 消费) ──
        self._target_mode = "end_effector"
        self._target_values = [0.0, 0.0, 0.0]
        self._target_updated = False
        self._return_to_zero = False

        # ── 主循环 → GUI (直接设置) ──
        self._current_position = [0.0, 0.0, 0.0]
        self._fps = 0.0
        self._interpolating = False
        self._ack_ok = True
        self._encoder_ok = False
        self._ik_error = 0.0
        self._max_rotation_error = 0.0
        self._input_source_name = ""
        self._active_mode = "end_effector"

    # ═══════════════════════════════════════════
    # GUI → 主循环
    # ═══════════════════════════════════════════

    def set_end_effector_target(self, x, y, z):
        with self._lock:
            self._target_mode = "end_effector"
            self._target_values = [float(x), float(y), float(z)]
            self._target_updated = True

    def set_rotation_target(self, rots):
        with self._lock:
            self._target_mode = "rotations"
            self._target_values = [float(r) for r in rots]
            self._target_updated = True

    def set_cable_length_target(self, lengths_mm):
        with self._lock:
            self._target_mode = "cable_length"
            self._target_values = [float(l) for l in lengths_mm]
            self._target_updated = True

    def set_curvature_target(self, t1_deg, p1_deg, t2_deg, p2_deg):
        with self._lock:
            self._target_mode = "curvature"
            self._target_values = [
                float(t1_deg), float(p1_deg),
                float(t2_deg), float(p2_deg),
            ]
            self._target_updated = True

    def request_return_to_zero(self):
        with self._lock:
            self._return_to_zero = True

    def consume_target_update(self):
        """消费目标更新。若自上次消费后有新目标，返回 (mode, values)；否则返回 None。"""
        with self._lock:
            if self._target_updated:
                self._target_updated = False
                return (self._target_mode, list(self._target_values))
            return None

    def consume_return_to_zero(self):
        """消费回零指令。若已请求，返回当前模式名；否则返回 None。"""
        with self._lock:
            if self._return_to_zero:
                self._return_to_zero = False
                return self._target_mode
            return None

    # ═══════════════════════════════════════════
    # 主循环 → GUI
    # ═══════════════════════════════════════════

    def set_pose(self, pos):
        with self._lock:
            self._current_position = list(pos)

    def get_pose(self):
        with self._lock:
            return list(self._current_position)

    def set_status(self, fps, interpolating, ack_ok, encoder_ok,
                   ik_error, max_rot_err, source_name, active_mode):
        with self._lock:
            self._fps = fps
            self._interpolating = interpolating
            self._ack_ok = ack_ok
            self._encoder_ok = encoder_ok
            self._ik_error = ik_error
            self._max_rotation_error = max_rot_err
            self._input_source_name = source_name
            self._active_mode = active_mode

    def get_status(self):
        with self._lock:
            return {
                "fps": self._fps,
                "interpolating": self._interpolating,
                "ack_ok": self._ack_ok,
                "encoder_ok": self._encoder_ok,
                "ik_error": self._ik_error,
                "max_rotation_error": self._max_rotation_error,
                "input_source": self._input_source_name,
                "active_mode": self._active_mode,
            }
