# config.py -- 集中配置参数

import numpy as np
from dataclasses import dataclass, field


# =====================================================
# 串口配置
# =====================================================

@dataclass
class SerialConfig:
    port_main: str = "COM13"        # 上位机串口端口
    port_sim: str = "COM14"         # 模拟器串口端口（与 port_main 虚拟串口对相连）
    baudrate: int = 115200          # 波特率
    timeout: float = 0.001          # 读超时 (s)


# =====================================================
# 通信协议配置
# =====================================================

@dataclass
class ProtocolConfig:
    frame_header: bytes = b'\xAA\x55'   # 帧头标识
    frame_tail: bytes = b'\x0D\x0A'     # 帧尾标识
    frame_len: int = 39                 # 帧总长度 (字节): 2头 + 1命令 + 32数据 + 2CRC + 2尾
    cmd_target: int = 0x01              # 命令字：发送目标值
    cmd_stop: int = 0x02                # 命令字：急停
    cmd_query: int = 0x03              # 命令字：查询状态
    cmd_feedback: int = 0xA1            # 命令字：编码器反馈（下位机 -> 上位机）
    cmd_ack: int = 0x81                 # 命令字：应答确认（下位机 -> 上位机）
    crc_polynomial: int = 0xA001        # CRC16-Modbus 多项式


# =====================================================
# 机器人配置
# =====================================================

@dataclass
class RobotConfig:
    # --- 第一部分几何参数 ---
    section1_length: float = 0.123       # 第一部分单节段长 L1 (m)
    section1_radius: float = 0.028       # 第一部分绳分布半径 r1 (m)
    section1_segments: int = 6          # 第一部分子节数 n（n 个相同 PCC 段串联）

    # --- 第二部分几何参数 ---
    section2_length: float = 0.123       # 第二部分单节段长 L2 (m)
    section2_radius: float = 0.028      # 第二部分绳分布半径 r2 (m)
    section2_segments: int = 4          # 第二部分子节数 m（m 个相同 PCC 段串联）

    num_cables: int = 8                 # 绳/腱总数（每节 4 根，两节共 8 根）

    # --- 逆运动学求解器参数 ---
    ik_initial_guess_deg: tuple = (10.0, 0.0, 10.0, 0.0)  # IK 初始猜测 [theta1, phi1, theta2, phi2] (度)，每节弯曲角
    ik_damping: float = 1e-3            # 阻尼最小二乘法的阻尼系数 lambda
    ik_max_iter: int = 50               # IK 最大迭代次数
    ik_tolerance: float = 1e-4          # IK 收敛容差 (m)，0.1mm 精度
    ik_convergence_tol: float = 1e-3   # IK 结果验收容差 (m)，超过此值拒绝下发
    jacobian_step: float = 1e-5         # 数值雅可比矩阵的有限差分步长 (rad)

    # --- 弯曲角安全限制 ---
    section1_theta_max_deg: float = 45.0    # 第一部分单节最大弯曲角 (度)
    section2_theta_max_deg: float = 45.0    # 第二部分单节最大弯曲角 (度)

    # --- 驱动器参数 ---
    spool_diameter: float = 0.01        # 绳盘直径 (m)，绳缠绕在舵机轴上的圆盘直径


# =====================================================
# 视觉配置
# =====================================================

@dataclass
class VisionConfig:
    tag_size: float = 0.05              # AprilTag 物理边长 (m)，即打印出的实际尺寸
    camera_matrix: np.ndarray = field(  # 相机内参矩阵 3x3，实际工程中需替换为标定结果
        default_factory=lambda: np.array([
            [800.0,   0.0, 320.0],      # fx, 0, cx
            [  0.0, 800.0, 240.0],      # 0, fy, cy
            [  0.0,   0.0,   1.0]       # 0, 0, 1
        ], dtype=np.float32)
    )
    dist_coeffs: np.ndarray = field(    # 畸变系数，默认无畸变
        default_factory=lambda: np.zeros((5, 1), dtype=np.float32)
    )
    aruco_dict: str = "DICT_APRILTAG_36h11"  # AprilTag 字典类型
    camera_index: int = 0               # 摄像头编号（0 通常是自带摄像头）

    # ---- 相机坐标系 → 机器人坐标系变换 ----
    #
    # 机器人坐标系定义（由绳布局决定）：
    #
    #   绳分布俯视图（从上往下看）：
    #
    #          绳2 (Y+)
    #           |
    #   绳1 ----+---- 绳3 (X+)
    #  (X-)     |
    #          绳4 (Y-)
    #
    #   绳1 (0°)   → X 负方向
    #   绳2 (90°)  → Y 正方向
    #   绳3 (180°) → X 正方向
    #   绳4 (270°) → Y 负方向
    #
    #   原点：机器人基座中心（绳分布圆中心）
    #   X 轴：绳1 → 绳3 方向（正方向指向绳3）
    #   Y 轴：绳4 → 绳2 方向（正方向指向绳2）
    #   Z 轴：竖直向上（右手定则）
    #
    # 变换公式：pose_robot = cam_to_robot_R @ pose_cam + cam_to_robot_t
    #
    # 当前为单位变换（待标定后替换）：
    #   cam_to_robot_R = I（无旋转）
    #   cam_to_robot_t = [0, 0, 0]（无平移）
    #
    cam_to_robot_R: np.ndarray = field(
        default_factory=lambda: np.eye(3, dtype=np.float64)
    )
    cam_to_robot_t: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )


# =====================================================
# 安全配置
# =====================================================

@dataclass
class SafetyConfig:
    # --- 障碍物检测 ---
    obstacle_awareness: bool = False        # 是否启用障碍物检测（集成到 IK 求解中）
    obstacle_zones: list = field(default_factory=lambda: [
        # 每个区域: {"center": [x,y,z], "radius": r}
        # 占位符，后续替换为真实障碍物地图
        {"center": [0.0, 0.0, 0.5], "radius": 0.05}
    ])

    # --- 目标位置变化限制 ---
    max_position_change: float = 0.02       # 单步最大位置变化量 (m)

    # --- 绳长变化阈值 ---
    max_cable_delta: float = 0.05           # 单步最大绳长变化 (圈)

    # --- IK 惩罚函数权重 ---
    obstacle_penalty_weight: float = 0.01   # 障碍物排斥势场权重
    theta_penalty_weight: float = 0.01      # 弯曲角惩罚权重
    cable_penalty_weight: float = 0.01      # 绳长惩罚权重
    theta_soft_ratio: float = 0.8           # 弯曲角软边界比例 (80% 处开始惩罚)
    cable_soft_max: float = 5.0             # 绳长软限制 (圈)，超过此值惩罚
    obstacle_margin: float = 1.5            # 障碍物安全裕度系数 (d_safe = radius * margin)
    min_displacement_weight: float = 0.01   # 绳位移正则化权重 (DLS步进中引导选绳位移小的方向)


# =====================================================
# 输入配置
# =====================================================

@dataclass
class InputConfig:
    # --- 输入模式 ---
    input_mode: str = "manual"              
    # "vision" | "manual" | "trajectory"

    # --- 手动模式子类型 ---
    manual_submode: str = "end_effector"       
    # "end_effector" | "rotations" | "cable_length" | "curvature"

    # 末端模式
    manual_end_effector: tuple = (0.10, 0.05, 1.10)  # [x, y, z] (m) — 远离 Z 轴可达点

    # 圈数模式
    manual_rotations: tuple = (0.0,) * 8    # 8 个舵机目标圈数

    # 绳长模式
    manual_cable_length: tuple = (0.0,) * 8 # 8 绳位移 (m)

    # 常曲率模式
    manual_curvature: tuple = (20.0, 0.0, 20.0, 180.0)  # [θ1, φ1, θ2, φ2] (度)

    # --- 轨迹模式参数 ---
    trajectory_type: str = "sine"           # "sine" | "circle" | "line"
    trajectory_amplitude: float = 0.05      # 轨迹幅度 (m)
    trajectory_frequency: float = 0.5       # 轨迹频率 (Hz)
    trajectory_center: tuple = (0.0, 0.0, 0.5)  # 轨迹中心 [x, y, z] (m)
    trajectory_axis: str = "x"              # 轨迹运动轴 ("x" | "y" | "z" | "xy")


# =====================================================
# 可视化配置
# =====================================================

@dataclass
class VisConfig:
    enable_opencv_vis: bool = True           # 是否启动 OpenCV 可视化线程
    enable_mujoco_vis: bool = True          # 是否启动 MuJoCo 可视化线程


# =====================================================
# 控制配置
# =====================================================

@dataclass
class ControlConfig:
    main_loop_hz: float = 50.0          # 主控制循环频率 (Hz)
    send_hz: float = 100.0              # 串口发送频率 (Hz)
    pid_hz: float = 1000.0              # 下位机 PID 控制频率 (Hz)，仅模拟器使用
    feedback_hz: float = 20.0           # 编码器反馈发送频率 (Hz)，仅模拟器使用
    pid_gain: float = 0.01              # PID 比例增益 Kp，仅模拟器使用
    slow_loop_warn_sec: float = 0.025   # 主循环耗时超过此值时发出警告 (s)
    ack_timeout_sec: float = 0.2        # ACK 超时阈值，超过此时间未收到 ACK 则报警 (s)

    # --- 线程降频因子 ---
    vision_update_div: int = 5           # 视觉/IK 更新频率 = 主循环 / N (默认 50/5=10Hz)
    mujoco_update_div: int = 3           # MuJoCo 更新频率 = 主循环 / M (默认 50/3≈17Hz)

# =====================================================
# 模块级单例
# =====================================================

serial_cfg = SerialConfig()
protocol_cfg = ProtocolConfig()
robot_cfg = RobotConfig()
vision_cfg = VisionConfig()
safety_cfg = SafetyConfig()
input_cfg = InputConfig()
vis_cfg = VisConfig()
control_cfg = ControlConfig()
