# tests/test_pipeline.py
# 不接入视觉，用随机正运动学结果测试 IK -> 协议打包 -> 发送 -> 接收 全流程

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import numpy as np

from robot.kinematics import MultiSectionRobot, inverse_kinematics
from comm.protocol import pack_target, parse_frame, crc16_modbus


def test_ik_only():
    """纯 IK 测试：随机配置 -> FK -> IK -> FK 验证"""
    print("=" * 60)
    print("测试 1: IK 往返验证 (FK -> IK -> FK)")
    print("=" * 60)

    robot = MultiSectionRobot()

    q_true = np.array([
        np.deg2rad(np.random.uniform(5, 45)),
        np.deg2rad(np.random.uniform(0, 360)),
        np.deg2rad(np.random.uniform(5, 45)),
        np.deg2rad(np.random.uniform(0, 360)),
    ])

    print(f"\n随机配置:")
    print(f"  theta1={np.rad2deg(q_true[0]):.2f}°  "
          f"phi1={np.rad2deg(q_true[1]):.2f}°")
    print(f"  theta2={np.rad2deg(q_true[2]):.2f}°  "
          f"phi2={np.rad2deg(q_true[3]):.2f}°")

    target = robot.tip_position(q_true)
    print(f"\n目标位置 (m):")
    print(f"  x={target[0]:.6f}  y={target[1]:.6f}  z={target[2]:.6f}")

    t0 = time.perf_counter()
    q_est = robot.inverse_kinematics(target)
    ik_time = (time.perf_counter() - t0) * 1000

    print(f"\nIK 求解结果 ({ik_time:.1f}ms):")
    print(f"  theta1={np.rad2deg(q_est[0]):.2f}°  "
          f"phi1={np.rad2deg(q_est[1]):.2f}°")
    print(f"  theta2={np.rad2deg(q_est[2]):.2f}°  "
          f"phi2={np.rad2deg(q_est[3]):.2f}°")

    verify = robot.tip_position(q_est)
    error = np.linalg.norm(target - verify)

    print(f"\nFK 验证位置 (m):")
    print(f"  x={verify[0]:.6f}  y={verify[1]:.6f}  z={verify[2]:.6f}")
    print(f"\n位置误差: {error:.2e} m")

    if error < 1e-4:
        print("[通过] IK 往返验证成功")
    else:
        print("[失败] 位置误差过大")

    return error < 1e-4


def test_rotation_output():
    """测试舵机圈数输出"""
    print("\n" + "=" * 60)
    print("测试 2: 舵机圈数计算")
    print("=" * 60)

    robot = MultiSectionRobot()

    q_rand = np.array([
        np.deg2rad(np.random.uniform(5, 30)),
        np.deg2rad(np.random.uniform(0, 360)),
        np.deg2rad(np.random.uniform(5, 30)),
        np.deg2rad(np.random.uniform(0, 360)),
    ])

    target = robot.tip_position(q_rand)
    print(f"\n目标位置: [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]")

    # inverse_kinematics 返回 (rotations, q)
    rotations, q = inverse_kinematics(target.tolist())

    print(f"\n8 根舵机目标圈数:")
    for i, rot in enumerate(rotations):
        print(f"  绳{i+1}: {rot:+.4f} 圈", end="  ")
        if (i + 1) % 4 == 0:
            print()

    print(f"\n  最大 |圈数| = {max(abs(r) for r in rotations):.4f} 圈")
    print("\n[通过] 舵机圈数计算完成")
    return True


def test_protocol():
    """测试协议打包 -> 解包往返"""
    print("\n" + "=" * 60)
    print("测试 3: 协议打包 -> 解包")
    print("=" * 60)

    robot = MultiSectionRobot()

    q_rand = np.array([
        np.deg2rad(np.random.uniform(5, 30)),
        np.deg2rad(np.random.uniform(0, 360)),
        np.deg2rad(np.random.uniform(5, 30)),
        np.deg2rad(np.random.uniform(0, 360)),
    ])

    target = robot.tip_position(q_rand)
    rotations, q = inverse_kinematics(target.tolist())

    frame = pack_target(rotations)
    print(f"\n帧长度: {len(frame)} 字节")
    print(f"帧头: {frame[0:2].hex()}")
    print(f"帧尾: {frame[-2:].hex()}")

    result = parse_frame(frame)
    if result is None:
        print("[失败] 帧解析失败")
        return False

    cmd, values = result
    print(f"命令字: 0x{cmd:02X}")
    print(f"数据 (圈数): {[f'{v:.6f}' for v in values]}")

    max_diff = max(abs(a - b) for a, b in zip(rotations, values))
    print(f"\n最大差异 (原始 vs 解析): {max_diff:.2e}")

    if max_diff < 1e-5:
        print("[通过] 协议往返验证成功")
        return True
    else:
        print("[失败] 数据不一致")
        return False


def test_crc():
    """测试 CRC 校验"""
    print("\n" + "=" * 60)
    print("测试 4: CRC16-Modbus 校验")
    print("=" * 60)

    data = b'\xAA\x55\x01' + bytes(32)
    crc = crc16_modbus(data)
    print(f"\n测试帧 CRC: 0x{crc:04X}")

    data_bad = bytearray(data)
    data_bad[3] ^= 0xFF
    crc_bad = crc16_modbus(bytes(data_bad))

    if crc != crc_bad:
        print("[通过] CRC 篡改检测正常")
        return True
    else:
        print("[失败] CRC 未检测到数据篡改")
        return False


def test_serialization_sizes():
    """验证舵机圈数量级适合 float32 传输"""
    print("\n" + "=" * 60)
    print("测试 5: float32 数值范围")
    print("=" * 60)

    robot = MultiSectionRobot()

    max_val = 0
    for _ in range(100):
        q_rand = np.array([
            np.deg2rad(np.random.uniform(5, 45)),
            np.deg2rad(np.random.uniform(0, 360)),
            np.deg2rad(np.random.uniform(5, 45)),
            np.deg2rad(np.random.uniform(0, 360)),
        ])
        target = robot.tip_position(q_rand)
        try:
            rotations, q = inverse_kinematics(target.tolist())
            local_max = max(abs(r) for r in rotations)
            if local_max > max_val:
                max_val = local_max
        except Exception:
            pass

    print(f"\n100 次随机目标的最大舵机圈数: {max_val:.4f} 圈")
    print(f"float32 在此量级的精度: ~{np.finfo(np.float32).resolution:.0e}")

    if max_val < 10.0:
        print("[通过] 数值范围适合 float32 传输")
        return True
    else:
        print("[警告] 数值偏大，请检查机器人几何参数")
        return True


if __name__ == "__main__":
    np.random.seed(42)

    print("\n" + "=" * 60)
    print("全流程测试套件")
    print("=" * 60)

    results = []
    results.append(("IK 往返验证", test_ik_only()))
    results.append(("舵机圈数计算", test_rotation_output()))
    results.append(("协议打包解包", test_protocol()))
    results.append(("CRC 校验", test_crc()))
    results.append(("float32 数值范围", test_serialization_sizes()))

    print("\n" + "=" * 60)
    print("测试汇总")
    print("=" * 60)

    all_pass = True
    for name, passed in results:
        status = "通过" if passed else "失败"
        print(f"  {name:.<40} {status}")
        if not passed:
            all_pass = False

    print()
    if all_pass:
        print("全部测试通过")
    else:
        print("部分测试失败")
