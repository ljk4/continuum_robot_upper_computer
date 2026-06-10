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

### 运行测试（无需硬件和串口）

```bash
cd 大作业
python tests/test_pipeline.py
```

### 仿真模式（无需硬件）

需要虚拟串口对（如 com0com），将 COM13 和 COM14 连接。

```bash
# 终端 1：启动模拟下位机
python sim/fake_stm32.py

# 终端 2：启动上位机
python main.py
```

### MuJoCo 可视化

### 仿真验证（无硬件，推荐）

通过 MuJoCo 3D 渲染全流程闭环验证：输入源 → IK → EMA → 插值器 → tendon_to_config → FK，与实物控制管线一致。
```bash
python tests/test_mujoco.py
```

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

## 数据流

### 主控制管线 (main.py, 50Hz)

```
                    ┌──────────────┐
                    │  input/      │  视觉 (AprilTag) / 手动 / 轨迹
                    │  sources.py  │
                    └──────┬───────┘
                           │  target=[x,y,z]  或  rotations=[r1..r8]
                           │  (每 5 帧 = 10Hz)
                           ▼
                    ┌──────┴──────┐
                    │   模式判断   │
                    └──────┬──────┘
                           │
          ┌────────────────┼────────────────┐
          ▼                                 ▼
   get_direct_rotations()            get_target() != None
   返回预计算圈数                    返回末端位置
   (绳长/圈数/曲率模式)              (末端/视觉/轨迹模式)
          │                                 │
          │                                 ▼
          │              ┌──────────────────────────────┐
          │              │  IK 求解器                    │
          │              │  inverse_kinematics()         │
          │              │                               │
          │              │  damped least squares         │
          │              │  (JᵀJ + wJ_tᵀJ_t + λI)dq = Jᵀ·error │
          │              │  w=min_displacement_weight      │
          │              │                               │
          │              │  + 障碍物排斥势场 ──┐         │
          │              │  + 弯曲角梯度惩罚    │ 惩罚函数 │
          │              │  + 绳长梯度惩罚      │         │
          │              │  + 最小位移正则化  ──┘         │
          │              │                               │
          │              │  clamp_theta 硬夹紧兜底        │
          │              │  ||FK(q)-target|| 收敛检查     │
          │              └──────────────┬───────────────┘
          │                             │
          │              ┌──────────────┴───────────────┐
          │              │  q=[θ1,φ1,θ2,φ2]             │
          │              │       ↓ config_to_all_tendons │
          │              │  8 绳位移 dl (m)              │
          │              │       ↓ ÷(π·D), 带负号        │
          │              │  rotations=[r1..r8] (圈)      │
          │              └──────────────┬───────────────┘
          │                             │
          └──────────┬──────────────────┘
                     │  raw_rotations (8 floats)
                     ▼
          ┌──────────────────────┐
          │  EMA 低通滤波         │  α=0.3, 首次直接初始化
          │  smoothed_rotations  │
          └──────────┬───────────┘
                     │
                     ▼
          ┌──────────────────────┐
          │  绳长插值器           │  max_cable_delta=0.05圈/步
          │  RotationInterpolator│  每帧步进 → 远目标平滑逼近
          │  update_target()     │  不二值拒绝，插值器自带限幅
          │  get_next_step()     │
          └──────────┬───────────┘
                     │  rotations (每帧, ~50Hz)
                     ▼
          ┌──────────────────────┐
          │  SenderThread        │  100Hz 独立线程
          │  pack_target() → 串口 │  39字节定长帧 + CRC16
          └──────────┬───────────┘
                     │  UART
                     ▼
          ┌──────────────────────┐
          │  STM32 下位机         │  PID 1kHz
          │  编码器反馈 @ 20Hz    │  ACK 应答
          └──────────┬───────────┘
                     │  反馈帧 (0xA1) / ACK (0x81)
                     ▼
          ┌──────────────────────┐
          │  ReceiverThread      │  独立线程
          │  解析反馈 + ACK 超时  │  200ms 无 ACK → 急停
          └──────────────────────┘
```

### 三空间映射

```
  驱动空间                  配置空间                   任务空间
  (8维)                    (4维)                      (3维)
  
  rotations[0..7]  ←──→  q=[θ1,φ1,θ2,φ2]  ←──→  pos=[x,y,z]
  舵机圈数               PCC 弯曲参数             末端位置
  
        config_to_all_tendons(q)             FK: tip_position(q)
  圈数 ──────────────────────> 绳位移            q ──────────────> pos
        rotations = -dl/(π·D)
  
        tendon_to_config(tendons)              IK: inverse_kinematics(pos)
  绳位移 ─────────────────────> q              pos ─────────────────> q
        (阻尼最小二乘, 8×4→4)                   (阻尼最小二乘, 3→4)

  腱位移(单节):
    绳1: dl=-r·θ·cos(φ)    绳2: dl=+r·θ·sin(φ)
    绳3: dl=+r·θ·cos(φ)    绳4: dl=-r·θ·sin(φ)
    
  总位移(含耦合):
    绳1-4 = n·dl(θ1,φ1)
    绳5-8 = m·dl(θ2,φ2) + n·dl(θ1,φ1)   ← 穿过第一部分的路径耦合
```

### IK 求解器内部流程

```
  target=[x,y,z], q0(上次解)
       │
       ▼
  ┌─────────────────────────────────────┐
  │  迭代 (max 50次)                     │
  │                                      │
  │  ① FK(q) → pos, error=target-pos    │
  │  ② ||error|| < tol ? → 收敛退出      │
  │  ③ J = jacobian(q)     (3×4 数值)   │
  │  ④ (JᵀJ+wJ_tᵀJ_t+λI)dq = Jᵀ·error       │
  │     含绳位移正则化, 偏好位移小的方向    │
  │  ⑤ dq_penalty = compute_penalty():   │
  │     · 障碍物: Jᵀ·force_repulsive     │
  │     · 弯曲角: -∇P_theta              │
  │     · 绳长:   -J_tendonᵀ·∇P_cable   │
  │     · 最小位移: -w·J_tendonᵀ·tendons │
  │     (外部偏置叠加, 不经A⁻¹, 避零空间放大)│
  │  ⑥ dq = dq_task + dq_penalty        │
  │  ⑦ 线搜索: α=1→0.5→…→0.0625 (最多5次)│
  │     总代价=位置²+w·Σtendons²+penalty │
  │     总代价下降则接受, 否则继续        │
  │  ⑧ clamp_theta(q)  硬夹紧兜底       │
  │                                      │
  └──────────────────┬──────────────────┘
                     │  q
                     ▼
  ┌─────────────────────────────────────┐
  │  验收: ||FK(q) - target||            │
  │  超过 ik_convergence_tol → 返回 None │
  │  通过 → 返回 (rotations, q)          │
  └─────────────────────────────────────┘
```

### 安全约束分层

```
                   ┌──────────────────────────┐
  不可违反         │  θ ≤ θ_max (见 config.py) 硬夹紧 (clamp)   │  IK 迭代最后一步
  (硬约束)         │  IK 收敛检查 (1mm)         │  不满足 → 返回 None
                   └──────────────────────────┘
                   ┌──────────────────────────┐
  IK 内引导        │  障碍物排斥势场            │  80% 弯曲角处开始惩罚
  (软约束/惩罚)    │  弯曲角梯度惩罚            │  权重 0.01, 可在 config 调整
                   │  绳长梯度惩罚 (>5圈)       │
                  │  最小位移正则化            │  梯度偏置, 不经 A⁻¹ 避免零空间放大
                  │  线搜索: 总代价下降         │  安全约束不再被位置误差判据静默忽略
                   └──────────────────────────┘
                   ┌──────────────────────────┐
  时序平滑         │  位置限幅 (0.02m/步)       │  IK 更新周期 (10Hz)
  (插值/滤波)      │  EMA 低通 (α=0.3)         │
                   │  插值器 (0.05圈/步)        │  每帧 (50Hz)
                   └──────────────────────────┘
                   ┌──────────────────────────┐
  通信保护         │  ACK 超时 → 急停 (200ms)  │  main.py 主循环
                   │  CRC16 校验               │  串口协议层
                   └──────────────────────────┘
```

### 线程架构

```
  main thread (50Hz)
    │
    ├── SenderThread (100Hz)    ──→ 串口 TX
    ├── ReceiverThread          ←── 串口 RX
    ├── _VisionThread           ←── 摄像头采集
    ├── VisThread (OpenCV)      ──→ 屏幕显示
    └── MuJoCoVisThread (~60Hz) ──→ 3D 窗口

  fake_stm32 (独立进程, 仿真用)
    ├── pid_loop     (1kHz)     ←── 接收线程 → 串口
    ├── UartReceiver            ←── 串口
    ├── FeedbackSender (20Hz)   ──→ 串口
    └── StatusMonitor (1Hz)     ──→ 控制台
```

### 仿真验证数据流 (test_simulation.py vs main.py)

```
  main.py:                     test_simulation.py:
  ─────────                    ────────────────────
  interpolator.get_next_step()  ←── 相同 ──→  interpolator.get_next_step()
        │                                            │
  sender.update_target()        不同              tendon_to_config()
  串口 → STM32 → 电机                             (驱动→配置逆映射)
        │                                            │
        │                                       q_achieved = FK⁻¹(rots)
        │                                            │
        │                                       tip = FK(q_achieved)
        │                                            │
  mujoco_thread.update_state   ←── 不同 ──→  draw_scene(q_achieved)
  (显示 IK 解 q_ik)                           (显示仿真实际姿态)
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

## 运动学模型

### PCC 假设（分段常曲率）

采用 PCC（Piecewise Constant Curvature）模型，每段用两个配置变量描述：

- **theta** (θ) — **单节**弯曲角 (rad)，总弯曲角 = 段数 × θ
- **phi** (φ) — 弯曲平面角 (rad)

两节机器人共有 4 个配置变量：`q = [theta1, phi1, theta2, phi2]`

### 多段结构

```
基座 ─── 第一部分(n段) ─── 第二部分(m段) ─── 末端
  │         │                  │
  ├── 绳1 ──┤ 锚定             │
  ├── 绳2 ──┤ 锚定             │
  ├── 绳3 ──┤ 锚定             │
  ├── 绳4 ──┤ 锚定             │
  ├── 绳5 ────────────────────┤ 锚定（穿过第一部分）
  ├── 绳6 ────────────────────┤ 锚定
  ├── 绳7 ────────────────────┤ 锚定
  └── 绳8 ────────────────────┘ 锚定
```

### 求解链路

```
目标位置 [x, y, z]
    → IK 求解 (阻尼最小二乘) → q = [θ1, φ1, θ2, φ2]
    → 腱映射 (含耦合补偿) → 8 根绳位移 (m)
    → 圈数转换 → rotations = dl / (π × D)
    → 下发给下位机
```

### 正运动学

n 段级联：`T_total = T(θ1, φ1)^n @ T(θ2, φ2)^m`

### 腱位移

```
绳 1-4 总位移 = n × dl(θ1, φ1)
绳 5-8 总位移 = m × dl(θ2, φ2) + n × dl(θ1, φ1)   (含耦合)
```

### 逆运动学

阻尼最小二乘法（Damped Least Squares）：

```
(JᵀJ + w·J_tendonᵀJ_tendon + λI) dq = Jᵀ · error
其中 w=min_displacement_weight (默认 0.01), 引导求解器偏好绳位移小的方向
```

- J: 3×4 数值雅可比（有限差分，步长 1e-5 rad）
- λ: 阻尼系数 1e-3
- w: 绳位移正则化权重 0.01 (在 SafetyConfig.min_displacement_weight)
- 惩罚函数: 障碍物排斥势场 + 弯曲角梯度 + 绳长梯度 + 最小位移正则化
  (惩罚梯度作为外部偏置直接叠加到 dq, 不经过正规方程 A⁻¹ 缩放,
   避免在 J 零空间(3×4→1维零空间)方向被阻尼 λ⁻¹ ≈ 1000× 过度放大)
- 线搜索: 全步长优先，总代价(位置²+w·Σtendons²+penalty)增大时回溯减半（最多 5 次）
  设计意图: 仅检查位置误差会导致惩罚项(避障/限位)被静默忽略
- 收敛: ||error|| < 1e-4 m 或 50 次迭代
- clamp_theta 硬夹紧兜底（θ ≤ θ_max, 见 config.py）

### 安全机制

| 层级 | 机制 | 位置 | 说明 |
|------|------|------|------|
| 硬约束 | 弯曲角夹紧 | `robot/kinematics.py` | IK 迭代最后一步 clamp, θ ≤ θ_max (见 config.py) |
| 硬约束 | IK 收敛检查 | `robot/kinematics.py` | FK(q) 与目标偏差 > 1mm 则拒绝下发 |
| 软约束 | 障碍物排斥势场 | `robot/kinematics.py` | FK(q) 靠近障碍物时产生排斥力, 融入 DLS 梯度 |
| 软约束 | 弯曲角惩罚 | `robot/kinematics.py` | θ 超过 80% 限制时施加二次惩罚梯度 |
| 软约束 | 绳长惩罚 | `robot/kinematics.py` | 圈数超过 5 圈时通过腱雅可比回传惩罚梯度 |
| 软约束 | 绳位移正则化 | `robot/kinematics.py` | Hessian 含 J_tendonᵀJ_tendon, 梯度偏置在外部 dq_penalty 中, 避免 A⁻¹ 零空间放大 |
| 时序 | EMA 低通滤波 | `main.py` | α=0.3, 首次直接初始化 |
| 时序 | 绳长插值器 | `robot/safety.py` | 每步限幅 0.01 圈 (可配), 不拒绝远目标—逐步逼近 |
| 通信 | ACK 超时急停 | `main.py` | 200ms 无 ACK 自动发送急停帧并锁定位置 |
| 通信 | CRC16 校验 | `comm/protocol.py` | 帧错误静默丢弃 |

### 默认参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 第一部分段长 L1 / 段数 n | 0.123 m / 6 | 第一部分 |
| 第二部分段长 L2 / 段数 m | 0.123 m / 4 | 第二部分 |
| 绳分布半径 r | 0.028 m | 两部分相同 |
| 绳盘直径 D | 0.1 m | 舵机轴 |
| 阻尼系数 λ | 1e-3 | IK 求解器 |
| 绳位移正则化权重 w | 0.01 | Hessian + 外部梯度偏置, 避免 A⁻¹ 零空间放大 |
| 弯曲角上限 (单节) | 30° (可配) | config.section_theta_max_deg, clamp_theta 硬夹紧 |
| 惩罚权重 (θ/绳长/障碍物) | 0.01 | 软约束惩罚权重, 可独立调整 |
| 最大迭代 / 收敛容差 | 50 / 1e-4 m | IK 收敛条件 |
| 雅可比步长 | 1e-5 rad | 数值差分 |

## 扩展

- **替换视觉模块**：修改 `vision/tracker.py` 中的 `VisionTracker` 类
- **替换 IK 模块**：修改 `robot/kinematics.py` 中的 `inverse_kinematics()` 函数
- **修改机器人参数**：编辑 `config.py` 中的 `RobotConfig`
- **替换通信协议**：修改 `comm/protocol.py`

## 待办事项

- 零点问题
- 下位机速度规划（上位机高低频线程规划亦可）
- 调试：每次回零有误差
- 下位机舵机型号及能力确认
- 持续轨迹跟踪 vs 间断式跟踪
- 工作空间可达性检查
- S型轨迹与障碍物约束
