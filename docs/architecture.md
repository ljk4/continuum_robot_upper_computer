# 系统架构

> 从 README.md 拆分出来的详细架构文档。

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
          │  绳长插值器           │  max_cable_delta=0.1圈/步
          │  RotationInterpolator│  每帧步进；sync_current()
          │  update_target()     │  编码器反馈闭环跟踪
          │  get_next_step()     │  无编码器时退回开环步进
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
                   │  插值器 (max_cable_delta圈/步) │  每帧 (50Hz), 唯一平滑层
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
    ├── MuJoCoVisThread (~60Hz) ──→ 3D 窗口
    └── GUIThread (tkinter, ~10Hz) ──→ 控制面板

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

> **关键区别**：`main.py` 将圈数**原样下发**给硬件（不检查 PCC 约束）；
> `test_simulation.py` 使用 `tendon_to_config()` 反算姿态，
> 当圈数不满足 dl1=-dl3, dl2=-dl4 约束时，反算结果是最近似解（有残差）。
> 详见 [PCC 绳位移约束](#pcc-绳位移约束-为什么不能单独拉一根绳)。

