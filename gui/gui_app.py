# gui/gui_app.py — tkinter GUI 控制面板（独立线程）

import tkinter as tk
from tkinter import ttk
import threading
import numpy as np

from gui.shared_state import SharedState
from robot.kinematics import MultiSectionRobot, inverse_kinematics
from config import robot_cfg

UPDATE_MS = 100  # 10 Hz 刷新率


class GUIThread(threading.Thread):
    """tkinter 控制面板，运行在独立线程中。

    与主循环通过 SharedState 交换数据：
    - 用户输入的目标位置/圈数/绳长/曲率写入 SharedState
    - 主循环将当前位姿和状态写入 SharedState，GUI 定时刷新显示
    """

    def __init__(self, shared_state: SharedState):
        super().__init__(daemon=True)
        self._state = shared_state
        self._root = None
        self._running = True
        self._top_mode_var = None   # tk.StringVar: manual / vision / trajectory
        self._mode_var = None       # tk.StringVar: end_effector / rotations / ...
        self._submode_frame = None  # 手动子模式容器（vision/trajectory 时隐藏）
        self._input_frame = None    # 动态输入区域容器
        self._input_vars = {}
        self._conv_labels = {}
        self._pos_labels = {}
        self._status_labels = {}
        self._ext_label = None      # 视觉/轨迹模式状态标签
        self._fallback_after_id = None  # vision 回退定时器 ID
        self._robot = MultiSectionRobot()
        self._spool_circ = np.pi * robot_cfg.spool_diameter

    def stop(self):
        self._running = False
        if self._root is not None:
            try:
                self._root.quit()
            except Exception:
                pass

    def run(self):
        self._root = tk.Tk()
        self._root.title("Robot Control Panel")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.geometry("840x700")
        self._root.resizable(True, True)

        self._build_window()
        self._schedule_update()
        self._root.mainloop()

    def _on_close(self):
        self._running = False
        self._root.quit()

    # ═══════════════════════════════════════════
    # 窗口构建
    # ═══════════════════════════════════════════


    def _on_top_mode_change(self):
        top = self._top_mode_var.get()

        if top == "manual":
            self._cancel_fallback()
            self._input_frame.pack(fill="x", padx=8, pady=2)
            self._submode_frame.pack(
                fill="x", padx=8, pady=2, before=self._input_frame)
            self._ext_label.pack_forget()
            self._rebuild_input_fields()
            self._state.switch_top_mode("manual")

        elif top == "vision":
            self._fallback_after_id = None
            self._submode_frame.pack_forget()
            self._input_frame.pack_forget()
            self._ext_label.config(
                text="Vision: Trying to open camera...", foreground="gray")
            self._ext_label.pack(fill="x", padx=8, pady=8)
            self._state.switch_top_mode("vision")

        elif top == "trajectory":
            self._cancel_fallback()
            self._submode_frame.pack_forget()
            self._input_frame.pack_forget()
            self._ext_label.config(
                text="Trajectory: active (params in config.py)", foreground="blue")
            self._ext_label.pack(fill="x", padx=8, pady=8)
            self._state.switch_top_mode("trajectory")

    def _cancel_fallback(self):
        if self._fallback_after_id is not None:
            self._root.after_cancel(self._fallback_after_id)
            self._fallback_after_id = None

    def _check_vision_fallback(self):
        available, msg = self._state.get_vision_status()
        if self._top_mode_var.get() != "vision":
            return
        if not available:
            if self._fallback_after_id is None:
                self._ext_label.config(
                    text="Vision unavailable: " + msg
                    + " - reverting to Manual",
                    foreground="red")
                self._fallback_after_id = self._root.after(2000, lambda: (
                    self._top_mode_var.set("manual"),
                    self._on_top_mode_change()
                ))
        else:
            if self._fallback_after_id is not None:
                self._root.after_cancel(self._fallback_after_id)
                self._fallback_after_id = None
            self._ext_label.config(
                text="Vision: connected - " + msg, foreground="green")

    def _build_window(self):
        # ── 顶层模式选择 ──
        top_frame = ttk.LabelFrame(self._root, text="Mode", padding=8)
        top_frame.pack(fill="x", padx=8, pady=(8, 2))

        self._top_mode_var = tk.StringVar(value="manual")
        for text, val in [
            ("Manual (手动输入)", "manual"),
            ("Vision (摄像头 AprilTag)", "vision"),
            ("Trajectory (预设轨迹)", "trajectory"),
        ]:
            ttk.Radiobutton(
                top_frame, text=text, variable=self._top_mode_var,
                value=val, command=self._on_top_mode_change
            ).pack(anchor="w", pady=1)

        # ── 手动子模式（仅 manual 顶层模式时可见）──
        self._submode_frame = ttk.LabelFrame(self._root, text="Manual Submode", padding=8)
        self._submode_frame.pack(fill="x", padx=8, pady=2)

        self._mode_var = tk.StringVar(value="end_effector")
        for text, val in [
            ("End Effector  [X, Y, Z]", "end_effector"),
            ("Rotations     [R1..R8]", "rotations"),
            ("Cable Length  [L1..L8 mm]", "cable_length"),
            ("Curvature     [t1,p1,t2,p2 deg]", "curvature"),
        ]:
            ttk.Radiobutton(
                self._submode_frame, text=text, variable=self._mode_var,
                value=val, command=self._on_mode_change
            ).pack(anchor="w", pady=1)

        # ── 外部模式状态标签（vision/trajectory 时可见）──
        self._ext_label = ttk.Label(
            self._root, text="", font=("", 10),
            foreground="gray", anchor="center")
        # 初始隐藏，_on_top_mode_change 控制

        # ── 动态输入区域 ──
        self._input_frame = ttk.LabelFrame(self._root, text="Target Input", padding=8)
        self._input_frame.pack(fill="x", padx=8, pady=2)
        self._rebuild_input_fields()

        # ── 按钮 ──
        btn_frame = ttk.Frame(self._root, padding=6)
        btn_frame.pack(fill="x", padx=8, pady=2)

        ttk.Button(btn_frame, text="Set Target",
                   command=self._on_set_target).pack(side="left", padx=(0, 8))
        ttk.Button(btn_frame, text="Return to Zero",
                   command=self._on_return_zero).pack(side="left")

        # ── 三空间转换显示 ──
        conv_frame = ttk.LabelFrame(self._root, text="Target Conversion", padding=8)
        conv_frame.pack(fill="x", padx=8, pady=2)

        sections = [
            ("Task",     ["X (m)", "Y (m)", "Z (m)"]),
            ("Config",   ["t1(deg)", "p1(deg)", "t2(deg)", "p2(deg)"]),
            ("Actuation",["R1", "R2", "R3", "R4", "R5", "R6", "R7", "R8"]),
        ]
        for sec_name, fields in sections:
            row = ttk.Frame(conv_frame)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=sec_name + ":", width=10, anchor="e").pack(
                side="left", padx=(0, 4))
            texts = []
            for f in fields:
                texts.append(f + "=---")
            lbl = ttk.Label(row, text="  ".join(texts), anchor="w",
                            font=("Consolas", 8))
            lbl.pack(side="left", fill="x", expand=True)
            self._conv_labels[sec_name] = lbl

        # ── 当前位姿 ──
        pos_frame = ttk.LabelFrame(self._root, text="Current Position", padding=8)
        pos_frame.pack(fill="x", padx=8, pady=2)

        for i, axis in enumerate(["X", "Y", "Z"]):
            ttk.Label(pos_frame, text=f"{axis}:", width=2).grid(
                row=0, column=i * 2, sticky="e", padx=(4, 2), pady=2)
            lbl = ttk.Label(pos_frame, text="---", width=10, anchor="w",
                            font=("Consolas", 10))
            lbl.grid(row=0, column=i * 2 + 1, sticky="w", pady=2, padx=(0, 8))
            self._pos_labels[axis.lower()] = lbl

        # ── 状态指示 ──
        status_frame = ttk.LabelFrame(self._root, text="Status", padding=8)
        status_frame.pack(fill="both", expand=True, padx=8, pady=(2, 8))

        status_items = [
            ("fps",           "Loop FPS"),
            ("interpolating", "Interpolating"),
            ("ack",           "ACK"),
            ("encoder",       "Encoder"),
            ("ik_error",      "IK Error (m)"),
            ("rot_error",     "Max Rot Error"),
            ("source",        "Input Source"),
        ]
        for key, text in status_items:
            row = ttk.Frame(status_frame)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=text + ":", width=18, anchor="e").pack(
                side="left", padx=(0, 4))
            lbl = ttk.Label(row, text="---", anchor="w", font=("Consolas", 9))
            lbl.pack(side="left", fill="x", expand=True)
            self._status_labels[key] = lbl

        # 初始转换显示
        self._update_conversion()

    # ═══════════════════════════════════════════
    # 动态输入区域
    # ═══════════════════════════════════════════

    def _rebuild_input_fields(self):
        """根据当前模式销毁并重建输入控件。"""
        for w in self._input_frame.winfo_children():
            w.destroy()
        self._input_vars = {}

        mode = self._mode_var.get()

        if mode == "end_effector":
            self._build_xyz_fields()
        elif mode == "rotations":
            self._build_8value_fields("Rotation", "rev", [
                "#1", "#2", "#3", "#4", "#5", "#6", "#7", "#8",
            ])
        elif mode == "cable_length":
            self._build_8value_fields("Length", "mm", [
                "L1", "L2", "L3", "L4", "L5", "L6", "L7", "L8",
            ])
        elif mode == "curvature":
            self._build_curvature_fields()

    def _build_xyz_fields(self):
        """末端位置模式：X, Y, Z 三个输入框。"""
        for i, axis in enumerate(["X (m)", "Y (m)", "Z (m)"]):
            ttk.Label(self._input_frame, text=axis + ":", width=6).grid(
                row=0, column=i * 2, sticky="e", padx=(4, 2), pady=4)
            var = tk.StringVar(value="0.000")
            self._input_vars[axis[0].lower()] = var
            ttk.Entry(self._input_frame, textvariable=var, width=9).grid(
                row=0, column=i * 2 + 1, padx=(0, 6), pady=4)

    def _build_8value_fields(self, label_prefix, unit, names):
        """8 值模式（圈数/绳长）：4×2 网格。"""
        for i, name in enumerate(names):
            row = i // 4
            col = (i % 4) * 2
            ttk.Label(self._input_frame, text=f"{name}:", width=4).grid(
                row=row, column=col, sticky="e", padx=(4, 2), pady=2)
            var = tk.StringVar(value="0.000")
            self._input_vars[name] = var
            ttk.Entry(self._input_frame, textvariable=var, width=8).grid(
                row=row, column=col + 1, padx=(0, 8), pady=2)
        ttk.Label(self._input_frame, text=f"({label_prefix} in {unit})",
                  font=("", 8)).grid(row=2, column=0, columnspan=8,
                                     sticky="w", padx=4, pady=(0, 2))

    def _build_curvature_fields(self):
        """曲率模式：t1, p1, t2, p2 四个输入框（度）。"""
        items = [
            ("t1 (deg)", "t1"), ("p1 (deg)", "p1"),
            ("t2 (deg)", "t2"), ("p2 (deg)", "p2"),
        ]
        for i, (label, key) in enumerate(items):
            row = i // 2
            col = (i % 2) * 2
            ttk.Label(self._input_frame, text=label + ":", width=10).grid(
                row=row, column=col, sticky="e", padx=(4, 2), pady=4)
            var = tk.StringVar(value="0.0")
            self._input_vars[key] = var
            ttk.Entry(self._input_frame, textvariable=var, width=9).grid(
                row=row, column=col + 1, padx=(0, 14), pady=4)

    # ═══════════════════════════════════════════
    # 三空间转换
    # ═══════════════════════════════════════════

    def _update_conversion(self):
        """根据当前模式和输入值，计算并显示三个空间的转换结果。"""
        mode = self._mode_var.get()
        pos = [0.0, 0.0, 0.0]
        q_deg = [0.0, 0.0, 0.0, 0.0]
        rots = [0.0] * 8

        try:
            if mode == "end_effector":
                x = float(self._input_vars["x"].get())
                y = float(self._input_vars["y"].get())
                z = float(self._input_vars["z"].get())
                pos = [x, y, z]
                result = inverse_kinematics(pos)
                if result[0] is not None:
                    rots, q = result
                    q_deg = [np.rad2deg(v) for v in q]

            elif mode == "rotations":
                rots = [float(self._input_vars[f"#{i+1}"].get()) for i in range(8)]
                tendons = -np.array(rots, dtype=float) * self._spool_circ
                q = self._robot.tendon_to_config(tendons)
                pos = self._robot.tip_position(q).tolist()
                q_deg = [np.rad2deg(v) for v in q]

            elif mode == "cable_length":
                lengths = [float(self._input_vars[f"L{i+1}"].get()) for i in range(8)]
                rots = [-l / 1000.0 / self._spool_circ for l in lengths]
                tendons = -np.array(rots, dtype=float) * self._spool_circ
                q = self._robot.tendon_to_config(tendons)
                pos = self._robot.tip_position(q).tolist()
                q_deg = [np.rad2deg(v) for v in q]

            elif mode == "curvature":
                t1 = float(self._input_vars["t1"].get())
                p1 = float(self._input_vars["p1"].get())
                t2 = float(self._input_vars["t2"].get())
                p2 = float(self._input_vars["p2"].get())
                q_deg = [t1, p1, t2, p2]
                q = np.deg2rad(q_deg)
                pos = self._robot.tip_position(q).tolist()
                tendons = self._robot.config_to_all_tendons(q)
                rots = (-tendons / self._spool_circ).tolist()

        except (ValueError, Exception):
            pass  # 无效输入或 IK 失败时保持默认值

        self._conv_labels["Task"].config(
            text="  ".join(f"{f}={pos[i]:+.4f}" for i, f in
                           enumerate(["X", "Y", "Z"])))
        self._conv_labels["Config"].config(
            text="  ".join(f"{f}={q_deg[i]:+6.1f}" for i, f in
                           enumerate(["t1", "p1", "t2", "p2"])))
        self._conv_labels["Actuation"].config(
            text="  ".join(f"R{i+1}={rots[i]:+.3f}" for i in range(8)))

    # ═══════════════════════════════════════════
    # 回调
    # ═══════════════════════════════════════════

    def _on_mode_change(self):
        mode = self._mode_var.get()
        self._state.set_status(
            fps=0, interpolating=False, ack_ok=True, encoder_ok=False,
            ik_error=0.0, max_rot_err=0.0, source_name="GUI",
            active_mode=mode,
        )
        self._rebuild_input_fields()
        self._update_conversion()

    def _on_set_target(self):
        if self._top_mode_var.get() != "manual":
            return  # 视觉/轨迹模式不允许手动设置目标
        mode = self._mode_var.get()
        try:
            if mode == "end_effector":
                x = float(self._input_vars["x"].get())
                y = float(self._input_vars["y"].get())
                z = float(self._input_vars["z"].get())
                self._state.set_end_effector_target(x, y, z)

            elif mode == "rotations":
                rots = [float(self._input_vars[f"#{i+1}"].get())
                        for i in range(8)]
                self._state.set_rotation_target(rots)

            elif mode == "cable_length":
                lengths = [float(self._input_vars[f"L{i+1}"].get())
                           for i in range(8)]
                self._state.set_cable_length_target(lengths)

            elif mode == "curvature":
                t1 = float(self._input_vars["t1"].get())
                p1 = float(self._input_vars["p1"].get())
                t2 = float(self._input_vars["t2"].get())
                p2 = float(self._input_vars["p2"].get())
                self._state.set_curvature_target(t1, p1, t2, p2)

        except ValueError:
            pass

        self._update_conversion()

    def _on_return_zero(self):
        """一键回零：输入框归零 + 通知主循环。"""
        if self._top_mode_var.get() != "manual":
            return  # 视觉/轨迹模式不允许手动回零
        mode = self._mode_var.get()

        if mode == "end_effector":
            for k in ["x", "y", "z"]:
                self._input_vars[k].set("0.000")
            self._state.set_end_effector_target(0.0, 0.0, 0.0)

        elif mode == "rotations":
            for i in range(8):
                self._input_vars[f"#{i+1}"].set("0.000")
            self._state.set_rotation_target([0.0] * 8)

        elif mode == "cable_length":
            for i in range(8):
                self._input_vars[f"L{i+1}"].set("0.000")
            self._state.set_cable_length_target([0.0] * 8)

        elif mode == "curvature":
            for k in ["t1", "p1", "t2", "p2"]:
                self._input_vars[k].set("0.0")
            self._state.set_curvature_target(0.0, 0.0, 0.0, 0.0)

        self._state.request_return_to_zero()
        self._update_conversion()

    # ═══════════════════════════════════════════
    # 定时刷新 (10 Hz)
    # ═══════════════════════════════════════════

    def _schedule_update(self):
        self._refresh()
        if self._running:
            self._root.after(UPDATE_MS, self._schedule_update)

    def _refresh(self):
        """从 SharedState 读取最新数据并更新控件。"""
        self._check_vision_fallback()
        pos = self._state.get_pose()
        self._pos_labels["x"].config(text=f"{pos[0]:+8.4f}")
        self._pos_labels["y"].config(text=f"{pos[1]:+8.4f}")
        self._pos_labels["z"].config(text=f"{pos[2]:+8.4f}")

        s = self._state.get_status()
        self._status_labels["fps"].config(text=f"{s['fps']:.0f}")

        interp = s["interpolating"]
        self._status_labels["interpolating"].config(
            text="Active" if interp else "Idle",
            foreground="orange" if interp else "green")

        ack = s["ack_ok"]
        self._status_labels["ack"].config(
            text="OK" if ack else "TIMEOUT",
            foreground="green" if ack else "red")

        enc = s["encoder_ok"]
        self._status_labels["encoder"].config(
            text="OK" if enc else "N/A",
            foreground="green" if enc else "gray")

        self._status_labels["ik_error"].config(
            text=f"{s['ik_error']:.4f}")
        self._status_labels["rot_error"].config(
            text=f"{s['max_rotation_error']:.4f}")
        self._status_labels["source"].config(
            text=s["input_source"])
