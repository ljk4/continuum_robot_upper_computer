# 绳驱动并联机器人上位机控制系统

机器人结构设计课程大作业。支持四种控制模式：手动输入、视觉伺服、轨迹跟踪、图像探索。通过 UART 将目标值发送给 STM32 下位机，实现闭环控制。

## 目录结构

```
大作业/
├── README.md              -- 本文件
├── config.py              -- 集中配置参数（dataclass）
├── main.py                -- 主程序入口，50Hz 控制循环
├── continuum_fk.xml       -- MuJoCo 模型文件
│
├── robot/                 -- 运动学 + 安全约束
│   ├── kinematics.py      -- PCC 模型、正/逆运动学、Jacobian、腱映射
│   └── safety.py          -- 角度夹紧、插值器（编码器闭环同步）
│
├── comm/                  -- 串口通信协议栈
│   ├── protocol.py        -- 帧打包/解包/CRC16-Modbus
│   ├── serial_mgr.py      -- 线程安全串口管理
│   ├── sender.py          -- 100Hz 发送线程
│   └── receiver.py        -- 接收线程（编码器反馈 + ACK）
│
├── vision/                -- 视觉位姿检测
│   └── tracker.py         -- AprilTag 检测 + solvePnP + 可视化
│
├── input/                 -- 输入源抽象
│   └── sources.py         -- Manual/Vision/Trajectory/Search 四种输入源
│
├── gui/                   -- tkinter GUI 控制面板
│   ├── shared_state.py    -- 线程安全状态桥
│   └── gui_app.py         -- 面板主程序（模式切换/目标设置/状态显示）
│
├── vis/                   -- 3D 可视化
│   ├── mujoco_model.py    -- MuJoCo 场景定义 + PCC 节点计算
│   └── mujoco_vis.py      -- MuJoCo 可视化线程
│
├── utils/                 -- 日志模块
├── sim/                   -- STM32 模拟器
├── tests/                 -- 测试脚本
├── tools/                 -- 相机标定工具
└── docs/                  -- 详细文档
```

## 环境配置

```bash
conda activate continuum_robot

# Python 依赖
pip install numpy pyserial opencv-python mujoco
```

**Ubuntu/Linux 额外步骤**：
```bash
sudo apt install python3-tk
# 修改 config.py 中串口为 /dev/ttyACM0 或 /dev/ttyUSB0
```

## 功能演示

以下所有演示均从项目根目录运行，需先启动 GUI（`python main.py`）。

### 1. 无硬件仿真 — 手动控制模式

无需任何硬件，验证运动学管线。

```bash
# 终端 1：启动模拟下位机
python sim/fake_stm32.py

# 终端 2：启动上位机
python main.py
```

GUI 面板弹出后：
1. 默认在 **Manual → End Effector** 模式，输入 X/Y/Z 坐标
2. 点击 **Set Target** → 机器人开始运动
3. 观察 MuJoCo 3D 窗口中机器人末端趋近目标
4. 点击 **Return to Zero** → 机器人回到直线状态
5. 切换手动子模式（Rotations / Cable Length / Curvature），体验不同输入方式

### 2. 轨迹跟踪模式

无需硬件，机器人按预设轨迹运动。

1. 在 `config.py` 中确认轨迹参数（默认正弦波，频率 0.2Hz，幅度 0.05m）
2. GUI 切换到 **Trajectory** 模式
3. MuJoCo 窗口中机器人末端沿轨迹运动
4. 观察目标位姿实时变化，插值器平滑追踪

> **参数调优**：轨迹频率过高时机器人追不上，降低 `trajectory_frequency`。编码器反馈自动同步插值器 current，舵机跟不上时不会越跑越远。

### 3. 视觉伺服模式（需摄像头）

通过 AprilTag 实时检测，机器人末端跟踪标签。

1. 打印一个 `DICT_APRILTAG_36h11` 标签（边长 5cm）
2. GUI 切换到 **Vision** 模式
3. OpenCV 窗口弹出，显示 AprilTag 检测画面
4. 移动标签 → 机器人末端跟随标签移动
5. 如果没有摄像头，自动退回 Manual 模式

> **相机标定**（可选，提高精度）：
> ```bash
> cd tools/camera_calibration_tool
> python capture.py --camera 1    # 拍摄棋盘格照片（Space 拍照, ESC 退出）
> python calibration.py --image_size 1280x720 --mode calibrate --corner 7x10 --square 15
> # 将 camera_params.xml 中的参数填入 config.py 的 VisionConfig
> ```

### 4. 图像伺服搜索模式（需摄像头）

摄像头装在末端（眼在手上），纯像素偏差驱动，无需 FK/IK/标定。

1. 将摄像头固定在机器人末端
2. GUI 切换到 **Search** 模式
3. 将 AprilTag 放在摄像头视野内 → 机器人自动趋近标签
4. 标签居中且大小合适 → 机器人 **自动锁定**（死区 + 滞回，约 1.5 秒）
5. 移开标签 → 锁定清零，机器人进入 **扫描模式**，缓慢旋转寻找标签
6. 重新找到标签 → 恢复跟踪

### 5. 全流程仿真（无硬件，推荐体验完整管线）

```bash
python tests/test_simulation.py
```

MuJoCo 中展示：输入源 → IK → 插值器 → tendon_to_config → FK 的完整闭环。调试输出包含三空间（Task/Config/Actuation）的目标和实际值。

## GUI 控制面板

运行 `python main.py` 后自动弹出（`gui_cfg.enable_gui` 控制）。

![GUI面板](docs/gui界面.png)

### 顶层模式

| 模式 | 说明 | 需要硬件 |
|------|------|----------|
| **Manual** | 手动输入，4 种子模式（末端/圈数/绳长/曲率） | 无 |
| **Vision** | 眼在手外：固定摄像头检测 AprilTag，IK 伺服 | 摄像头 + Tag |
| **Trajectory** | 预设轨迹跟踪（正弦/圆/直线） | 无 |
| **Search** | 眼在手上：像素偏差直接伺服，Tag 丢失自动扫描 | 摄像头 + Tag |

### 界面功能

| 功能 | 说明 |
|------|------|
| 模式切换 | 顶层 RadioButton（Manual/Vision/Trajectory/Search），Manual 下显示子模式 |
| 目标设置 | 输入数值 → Set Target，主循环立即响应 |
| 一键回零 | Return to Zero 清零所有目标（仅 Manual 模式生效） |
| 三空间转换 | 实时显示输入目标在 Task / Config / Actuation 的值 |
| 状态监控 | 10Hz 刷新：当前位姿、FPS、插值、ACK、编码器、IK 误差 |
| 搜索锁定 | Search 模式下 Tag 居中持续 1.5s 后自动锁定，标移出后恢复跟踪 |

### 界面布局

```
┌───────────────────────────────────────┐
│  Mode: ○ Manual  ○ Vision            │
│        ○ Trajectory  ○ Search        │
├───────────────────────────────────────┤
│  Manual Submode (仅 Manual 可见)      │
│  ○ EndEffector  ○ Rotations  ...     │
├───────────────────────────────────────┤
│  Target Input (动态切换)              │
│  [Set Target]  [Return to Zero]      │
├───────────────────────────────────────┤
│  Target Conversion (Task/Config/Act)  │
├───────────────────────────────────────┤
│  Current Position (实时 X/Y/Z)        │
├───────────────────────────────────────┤
│  Status (FPS/ACK/Encoder/IK Error)    │
└───────────────────────────────────────┘
```

## 配置说明

所有参数集中在 `config.py`，修改后重启生效。

| 配置组 | 关键参数 | 默认值 |
|--------|----------|--------|
| `SerialConfig` | 串口端口、波特率 | COM13 / 115200 |
| `RobotConfig` | 段长、半径、IK 参数 | 0.123m / 0.028m |
| `VisionConfig` | AprilTag 尺寸、相机内参/分辨率 | 0.05m / 标定值 |
| `VisionConfig` | 相机参数（曝光/帧率） | 1280×720 @ 30fps |
| `SafetyConfig` | 绳长变化/弯曲角上限 | max_cable_delta=0.1, θ_max=30° |
| `InputConfig` | 输入模式/轨迹参数 | manual / sine 0.2Hz |
| `ControlConfig` | 线程频率、超时 | 50Hz / ACK 0.2s |
| `GuiConfig` | GUI 面板开关 | enable_gui = True |

## 通信协议

39 字节定长帧：`帧头(2B) + 命令(1B) + 数据(32B, 8×float32) + CRC16(2B) + 帧尾(2B)`

## 详细文档

| 文档 | 内容 |
|------|------|
| [系统架构](docs/architecture.md) | 数据流、三空间映射、IK 流程、安全约束、线程架构 |
| [运动学模型](docs/kinematics.md) | PCC 模型、正/逆运动学、腱位移约束、安全机制 |
| [相机标定工具](tools/camera_calibration_tool/) | 棋盘格拍照 + OpenCV 标定 |

## 扩展

- **替换视觉模块**：修改 `vision/tracker.py` 中的 `VisionTracker` 类
- **替换 IK 模块**：修改 `robot/kinematics.py` 中的 `inverse_kinematics()` 函数
- **修改机器人参数**：编辑 `config.py` 中的 `RobotConfig`
- **添加新输入模式**：继承 `input/sources.py` 中的 `InputSource` 基类
