#include "app_main.h"
#include "app_config.h"
#include "bsp_board.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_ir.h"
#include "bsp_uart.h"
#include "usart.h"
#include "app_comms.h"
#include "app_motor_control.h"
#include "app_line_follow.h"

#if AGV_ENABLE_MOTOR_TEST_MODE
#include "diag_motor.h"
#include "diag_ir.h"
#endif

static uint32_t s_last_telemetry_tick = 0;
static uint32_t s_heartbeat_counter = 0;

void App_Init(void) {
    BSP_UART_Init(&huart1);
    App_Comms_Init();

    BSP_Motor_Init();
    BSP_Encoder_Init();
    BSP_IR_Init();
    App_MotorControl_Init();
    App_LineFollow_Init();

#if AGV_ENABLE_MOTOR_TEST_MODE
    Diag_Motor_Init();
    Diag_IR_Init();
#endif

    AGV_LOG("AGV System Initialized (72MHz TB6612 UART1)");
}

void App_Loop(void) {
    App_Comms_Process();

#if AGV_ENABLE_MOTOR_TEST_MODE
    Diag_Motor_Process();
    Diag_IR_Process();
#endif

    App_LineFollow_Process();
    App_MotorControl_Process();

    uint32_t current_tick = bsp_time_get_ms();

#if AGV_LINE_FOLLOW_AUTO_START
    static bool s_auto_started = false;
    static uint32_t s_boot_tick = 0;
    static uint32_t s_last_arm_blink = 0;

    if (!s_auto_started) {
        if (s_boot_tick == 0) {
            s_boot_tick = (current_tick > 0) ? current_tick : 1;
        }
        uint32_t elapsed = current_tick - s_boot_tick;
        if (elapsed < AGV_LINE_FOLLOW_ARM_DELAY_MS) {
            /* Nhấp nháy nhanh LED PC13 (100ms) để cảnh báo đếm ngược an toàn */
            if (current_tick - s_last_arm_blink >= 100) {
                s_last_arm_blink = current_tick;
                BSP_LED_Toggle();
            }
        } else {
            s_auto_started = true;
            App_LineFollow_Start(AGV_LINE_FOLLOW_DEFAULT_RPM);
            AGV_LOG("[AUTO] Line Follow Armed & Started");
        }
    }
#endif

    /* Heartbeat định kỳ mỗi 1000ms: Nhấp nháy LED PC13 và gửi frame log nhị phân */
    if (current_tick - s_last_telemetry_tick >= 1000) {
        s_last_telemetry_tick = current_tick;
        s_heartbeat_counter++;

        /* Nhấp nháy LED PC13 để báo vi điều khiển đang chạy bình thường (chỉ khi không trong giai đoạn arming) */
#if !AGV_LINE_FOLLOW_AUTO_START
        BSP_LED_Toggle();
#else
        if (s_auto_started) {
            if (App_LineFollow_GetState() == LINE_STATE_IDLE) {
                /* Xe đã dừng an toàn do mất vạch hoặc kết thúc: giữ LED sáng liên tục */
                BSP_LED_SetState(LED_STATE_ON);
            } else {
                BSP_LED_Toggle();
            }
        }
#endif
        AGV_LOG("HB #%u (Tick: %lu ms)", (unsigned int)s_heartbeat_counter, (unsigned long)current_tick);
    }
}