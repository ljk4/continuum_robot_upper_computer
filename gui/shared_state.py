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

        # ── 程序停止标志 ──
        self._stop_requested = False

        # ── GUI → 主循环 (写入 + 消费) ──
        self._target_mode = "end_effector"
        self._target_values = [0.0, 0.0, 0.0]
        self._target_updated = False
        self._return_to_zero = False

        # 顶层模式切换: manual / vision / trajectory
        self._top_mode = "manual"
        self._top_mode_changed = False

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

        # 视觉/轨迹模式状态
        self._vision_available = False
        self._vision_message = ""
        self._external_target = [0.0, 0.0, 0.0]  # 外部输入源提供的目标位姿

    # ═══════════════════════════════════════════
    # 程序停止
    # ═══════════════════════════════════════════

    def request_stop(self):
        with self._lock:
            self._stop_requested = True

    def stop_requested(self):
        with self._lock:
            return self._stop_requested

    # ═══════════════════════════════════════════
    # 顶层模式切换
    # ═══════════════════════════════════════════

    def switch_top_mode(self, mode):
        """GUI 请求切换到 manual / vision / trajectory。"""
        with self._lock:
            self._top_mode = mode
            self._top_mode_changed = True

    def get_top_mode(self):
        with self._lock:
            return self._top_mode

    def consume_top_mode_change(self):
        """主循环消费模式切换请求。一次性的。"""
        with self._lock:
            if self._top_mode_changed:
                self._top_mode_changed = False
                return self._top_mode
            return None

    # ═══════════════════════════════════════════
    # GUI → 主循环 (目标设置，仅 manual 模式使用)
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

    def set_external_target(self, pos):
        """主循环写入外部输入源（视觉/轨迹）提供的目标位姿。"""
        with self._lock:
            self._external_target = list(pos)

    def get_external_target(self):
        with self._lock:
            return list(self._external_target)

    def set_vision_available(self, available, message=""):
        """主循环报告视觉模式是否成功启动。"""
        with self._lock:
            self._vision_available = available
            self._vision_message = message

    def get_vision_status(self):
        with self._lock:
            return (self._vision_available, self._vision_message)

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
