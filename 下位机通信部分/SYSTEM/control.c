#include "control.h"
#include "cmd_handler.h"

/* =====================================================
 * 控制模块
 *
 * 职责：
 *   1. 读取编码器实际位置
 *   2. 计算 PID 误差 = target - encoder
 *   3. 输出驱动电机（PWM / 舵机）
 *
 * 调用频率：主循环周期约 4ms，即 ~250Hz
 * 如需更高频率（1kHz），可将 control_update() 移至定时器中断
 * ===================================================== */

/* ---- PID 参数 ---- */
#define PID_KP      0.01f       /* 比例增益 */
#define PID_KI      0.0f        /* 积分增益（暂不用） */
#define PID_KD      0.0f        /* 微分增益（暂不用） */

#define MAX_OUTPUT  1.0f        /* 输出限幅（归一化，±1.0 对应满速） */
#define DEADBAND    1e-5f       /* 到位死区（圈），误差小于此值不动作 */


/* ---- 内部状态 ---- */
static uint8_t ctrl_enabled = 1;

/* =====================================================
 * 控制初始化
 *
 * 待实现：
 *   - 初始化编码器接口（定时器编码器模式 / ADC / I2C）
 *   - 初始化电机驱动（PWM 输出 / 舵机接口）
 *   - 设置 PID 参数
 *   - 将 encoder_rotations[] 初始化为编码器实际读数
 * ===================================================== */
void control_init(void)
{
    /* TODO: 初始化编码器硬件 */

    /* TODO: 初始化电机驱动硬件 */

    /* TODO: 读取编码器初始值写入 encoder_rotations[] */

    ctrl_enabled = 1;
}

/* =====================================================
 * PID 伺服更新
 *
 * 每次主循环调用，执行以下步骤：
 *   1. 读取编码器 → 更新 encoder_rotations[i]
 *   2. 误差 = target_rotations[i] - encoder_rotations[i]
 *   3. 若 |误差| < 死区，跳过
 *   4. PID 输出 = Kp * 误差
 *   5. 输出限幅
 *   6. 驱动电机
 *
 * 模拟模式: 一阶滞后逼近 (gain=0.05, τ≈80ms@250Hz)
 * 实物模式: 取消模拟行, 取消注释 PWM 输出即可
 * ===================================================== */
void control_update(void)
{
    uint8_t i;
    float error, output;

    if (!ctrl_enabled) {
        /* 急停状态：电机不动作 */
        return;
    }

    for (i = 0; i < NUM_CABLES; i++) {

        /* 步骤 1: 读取编码器实际位置 */
        /* TODO: encoder_rotations[i] = 读编码器(i); */

        /* 步骤 2: 计算位置误差 */
        error = target_rotations[i] - encoder_rotations[i];

        /* 步骤 3: 死区判断 */
        if (error > -DEADBAND && error < DEADBAND)
            continue;

        /* 步骤 4: P 控制 */
        output = PID_KP * error;

        /* 步骤 5: 输出限幅 */
        if (output > MAX_OUTPUT)
            output = MAX_OUTPUT;
        else if (output < -MAX_OUTPUT)
            output = -MAX_OUTPUT;

        /* 步骤 6: 驱动电机 / 模拟 */
        /* TODO: 有实物电机时, 设置 PWM 输出(i, output); */

        /* 模拟: 一阶滞后逼近 target, gain=0.05 → τ≈80ms */
        encoder_rotations[i] += (target_rotations[i]
                                 - encoder_rotations[i]) * 0.05f;
    }
}

/* =====================================================
 * 急停 / 使能
 *
 * en=0: 急停，保持当前目标不动，电机锁在当前位置
 * en=1: 使能，恢复 PID 控制，继续跟踪目标
 * ===================================================== */
void control_set_enable(uint8_t en)
{
    ctrl_enabled = en;

    if (!en) {
        /* 急停：不修改 target_rotations，保持最后收到的目标值
         * control_update() 停止 PID 计算，电机保持当前位置 */
        /* TODO: 电机保持力矩输出（不要松力，否则绳会松弛） */
    }
}
