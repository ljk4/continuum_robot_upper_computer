#include "sys.h"
#include "led.h"
#include "key.h"
#include "protocol.h"
#include "cmd_handler.h"
#include "control.h"
#include "usart.h"

int main(void)
{
    uint16_t loop_counter = 0;
    uint8_t key;
    uint8_t estop_active = 0;   /* 本机急停状态 */

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(168);
    uart_init(115200);
	OLED_Init(0);

    LED_Init();
    KEY_Init();
    cmd_handler_init();
    control_init();

    /* LED0 熄灭表示正常运行 */
    LED0 = 1;

    while (1)
    {
        /* ---- 1. 按键扫描 ---- */
        key = KEY_Scan(0);

        if (key == 1) {
            /* KEY0 (PE4): 急停 */
            estop_active = 1;
            control_set_enable(0);
            LED0 = 0;   /* LED0 亮表示急停状态 */
        }
        else if (key == 2) {
            /* WK_UP (PA0): 复位，恢复正常运行 */
            estop_active = 0;
            control_set_enable(1);
            LED0 = 1;   /* LED0 熄灭表示正常 */
        }

        /* ---- 2. 通信解析 ---- */
        cmd_handler_poll();

        /* ---- 3. 控制更新（急停时不执行） ---- */
        if (!estop_active) {
            control_update();
        }

        /* ---- 4. 20Hz 反馈 ---- */
        if (++loop_counter >= 50) {
            loop_counter = 0;
            cmd_handler_send_feedback();
        }

        delay_ms(1);
    }
}
