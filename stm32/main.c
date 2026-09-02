#include "sys.h"
#include "usart.h"
#include "motor.h"
#include "encoder.h"
#include "control_system.h"
#include "delay.h"

int main(void)
{
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    uart_init(115200);
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    PWM_Init(7199, 1);
    Set_Pwm(0, 0);

    printf("\r\n[STM32] v3 open-loop ready\r\n");

    while (1)
    {
        System_Control();
        delay_ms(100);
    }
}
