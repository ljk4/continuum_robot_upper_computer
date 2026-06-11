# 绳驱动并联机器人上位机控制系统

机器人结构设计课程大作业。通过摄像头检测 AprilTag 标记获取末端位置，经逆运动学计算绳位移，通过 UART 将目标值发送给 STM32 下位机，实现闭环控制。

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
│   └── safety.py          -- 角度夹紧、障碍物检测、位置限幅、绳长插值器
│
├── comm/                  -- 串口通信协议栈
│   ├── protocol.py        -- 帧打包/解包/CRC16-Modbus
│   ├── serial_mgr.py      -- 线程安全串口管理
│   ├── sender.py          -- 100Hz 发送线程
│   └── receiver.py        -- 接收线程（编码器反馈 + ACK）
│
├── vision/                -- 视觉位姿检测
│   └── tracker.py         -- AprilTag 检测 + solvePnP 位姿解算 + 可视化
│
├── input/                 -- 输入源抽象
│   └── sources.py         -- 视觉/手动/轨迹三种输入模式 + 工厂函数
│
├── gui/                   -- tkinter GUI 控制面板
│   ├── shared_state.py    -- 线程安全状态桥
│   └── gui_app.py         -- 面板主程序
│
├── vis/                   -- 3D 可视化
│   ├── mujoco_model.py    -- MuJoCo 场景定义 + PCC 节点计算
│   └── mujoco_vis.py      -- MuJoCo 可视化线程
│
├── utils/
│   └── logger.py          -- 日志模块（控制台 + 文件）
│
├── sim/
│   └── fake_stm32.py      -- STM32 下位机模拟器（无需硬件即可测试）
│
├── tests/
│   ├── test_pipeline.py   -- IK + 协议 + CRC 全流程测试
│   ├── test_comm.py       -- 串口逐帧通信测试
│   └── test_mujoco.py     -- MuJoCo 独立可视化演示
│
└── docs/
    ├── 通信方案选择.md      -- 通信架构分析
    ├── 下位机代码实现指南.md -- STM32 固件开发指南
    └── 硬件调试指南.md      -- 接线、坐标、调试步骤
```

## 环境配置

```bash
conda activate continuum_robot

# Python 依赖
pip install numpy pyserial opencv-python mujoco
```

## 快速开始

### 仿真验证（无硬件，推荐）

通过 MuJoCo 3D 渲染全流程闭环验证：输入源 → IK → 插值器 → tendon_to_config → FK，与实物控制管线一致。

```bash
python tests/test_simulation.py
```

![仿真界面](docs\mujoco仿真初始位置.png)

### 仿真模式（无需硬件）

需要虚拟串口对（如 com0com），将 COM13 和 COM14 连接。

```bash
# 终端 1：启动模拟下位机
python sim/fake_stm32.py

# 终端 2：启动上位机
python main.py
```

![设置目标](docs\设置目标.png)


### 真实硬件

1. 连接摄像头和 STM32
2. 修改 `config.py` 中的串口端口和相机参数
3. 运行 `python main.py`

## 配置说明

所有参数集中在 `config.py` 中，修改后重启程序生效。

| 配置组 | 关键参数 | 默认值 |
|--------|----------|--------|
| `SerialConfig` | 串口端口、波特率 | COM13 / 115200 |
| `ProtocolConfig` | 帧头尾、命令字、CRC | 0xAA55 / 39字节 |
| `RobotConfig` | 段长、半径、IK 参数 | 0.123m / 0.028m |
| `VisionConfig` | AprilTag 尺寸、相机内参 | 0.05m / 单位变换 |
| `ControlConfig` | 各线程频率、超时阈值 | 50Hz/100Hz/20Hz |
| `GuiConfig` | GUI 面板开关 | enable_gui = True |
| `InputConfig` | 输入模式/子模式/默认值 | manual / end_effector |

### 输入模式

系统支持三种顶层输入模式，通过 `config.py` 的 `input_mode` 设置：

| 模式 | 说明 | GUI 交互 |
|------|------|----------|
| **manual** | 手动输入（4 种子模式），详见下方 GUI 章节 | 支持，GUI 可随时覆盖目标 |
| **vision** | 摄像头 AprilTag 实时检测末端位姿 | 不支持（摄像头自动驱动），可修改代码启用 |
| **trajectory** | 预设轨迹（正弦/圆/直线），参数可配 | 不支持（轨迹自动生成），可修改代码启用 |

> **注意**：vision 和 trajectory 模式下，GUI 的状态显示面板仍然可用，但 Set Target / Return to Zero 按钮不会生效（目标由摄像头或轨迹生成器决定）。如需在 GUI 中同时支持这两种模式的手动覆盖，可后续扩展。

## GUI 控制面板

运行 `python main.py` 后自动弹出（可通过 `config.py` 中 `gui_cfg.enable_gui` 开关）。

![GUI面板](docs\gui界面.png)

### 功能

| 功能 | 说明 |
|------|------|
| **四种输入模式** | 末端位置 / 舵机圈数 / 绳长 / 常曲率，RadioButton 切换 |
| **目标设置** | 输入数值后点击 Set Target，主循环立即响应 |
| **一键回零** | Return to Zero 按钮将所有目标清零，机器人回归直线状态 |
| **三空间转换** | 实时显示输入目标在 Task / Config / Actuation 三空间的对应值 |
| **状态监控** | 10Hz 刷新：当前位姿、FPS、插值状态、ACK、编码器、IK 误差 |

### 界面布局

```
┌─────────────────────────────────┐
│  Mode: ○ EndEffector  ○ Rotations│
│        ○ CableLength  ○ Curvature│
├─────────────────────────────────┤
│  Target Input (动态切换)         │
│  [Set Target]  [Return to Zero] │
├─────────────────────────────────┤
│  Target Conversion (三空间)      │
├─────────────────────────────────┤
│  Current Position (实时位姿)     │
├─────────────────────────────────┤
│  Status (FPS/ACK/编码器等)       │
└─────────────────────────────────┘
```

## 通信协议

39 字节定长帧：

| 字段 | 长度 | 说明 |
|------|------|------|
| 帧头 | 2B | 0xAA 0x55 |
| 命令字 | 1B | 0x01=目标 / 0x02=停止 / 0x03=查询 |
| 数据 | 32B | 8 x float32（绳长或编码器值） |
| CRC16 | 2B | Modbus 校验 |
| 帧尾 | 2B | 0x0D 0x0A |

## 详细文档

- [系统架构](docs/architecture.md) — 数据流、三空间映射、IK 求解器流程、安全约束分层、线程架构、仿真对比
- [运动学模型](docs/kinematics.md) — PCC 模型、正/逆运动学、腱位移约束、安全机制、默认参数

## 扩展

- **替换视觉模块**：修改 `vision/tracker.py` 中的 `VisionTracker` 类
- **替换 IK 模块**：修改 `robot/kinematics.py` 中的 `inverse_kinematics()` 函数
- **修改机器人参数**：编辑 `config.py` 中的 `RobotConfig`
- **替换通信协议**：修改 `comm/protocol.py`

<!-- ## 待办事项

- 零点问题
- 下位机速度规划（上位机高低频线程规划亦可）
- 调试：每次回零有误差
- 下位机舵机型号及能力确认
- 持续轨迹跟踪 vs 间断式跟踪
- 工作空间可达性检查
- S型轨迹与障碍物约束 -->

