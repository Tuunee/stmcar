/* ==============================================================
 * control_system.c —— STM32F103C8 从机版 (v3: 开环直驱)
 *
 * v3: 放弃 PI 速度闭环, 改为开环——速度字节直接映射 PWM 占空比
 *     不读编码器, 不依赖 SysTick, 由 main 循环直接调用
 *     方向完全由协议方向位决定, 行为确定
 *
 * ⚠ delay.c 保持原样不动! 不需要删任何东西!
 * ============================================================== */
#include "control_system.h"
#include "motor.h"

/* ===== usart.c 中定义的协议帧变量 ===== */
extern unsigned char USART_RX_STA;
extern unsigned char Frame_L_dir, Frame_L_spd, Frame_R_dir, Frame_R_spd;

/* PWM 上限: TIM4 arr=7199, 6480 ≈ 90% 占空比 */
#define PWM_MAX     6480
/* 起步死区补偿: 最小占空比 (低于此电机不动) */
#define DUTY_MIN    1200
/* 失联阈值: 25 拍 × 100ms = 2.5s */
#define LOST_LIMIT  25

static int stopped  = 1;
static int lost_cnt = 0;
static int cmd_l = 0, cmd_r = 0;

/* 速度字节 (0~150) → PWM 占空比 */
static int Spd_To_Duty(unsigned char spd)
{
    int duty;
    if (spd == 0) return 0;
    duty = ((int)spd * PWM_MAX) / 150;
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    return duty;
}

void PI_Reset(void) { }

/* ==================== 核心: main 每 100ms 调一次 ==================== */
void System_Control(void)
{
    int dl, dr;

    if (USART_RX_STA) {
        USART_RX_STA = 0;
        lost_cnt = 0;
        stopped = 0;

        dl = Spd_To_Duty(Frame_L_spd);
        dr = Spd_To_Duty(Frame_R_spd);
        cmd_l = Frame_L_dir ? -dl : dl;
        cmd_r = Frame_R_dir ? -dr : dr;
    } else if (!stopped) {
        lost_cnt++;
        if (lost_cnt >= LOST_LIMIT) {
            stopped = 1;
            cmd_l = 0;
            cmd_r = 0;
            Set_Pwm(0, 0);
        }
    }

    if (stopped) return;

    Set_Pwm(cmd_l, cmd_r);
}
