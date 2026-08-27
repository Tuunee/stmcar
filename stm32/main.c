#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include "motor.h"
#include "control_system.h"
#include "nfc.h"
#include <stdio.h>

int main(void)
{
    Stm32_Clock_Init(9);            // =====????8Mhz 9?? 8*9= 72mhz??72mhz
    MY_NVIC_PriorityGroupConfig(2); // =====???????
    uart_init(115200);              // =====??????115200
    USART2_Init(115200);              // NFC module (PN532 UART) @
    JTAG_Set(JTAG_SWD_DISABLE);     // =====??JTAG??
    JTAG_Set(SWD_ENABLE);           // =====??SWD?? ???????SWD????

    Encoder_Init_TIM2();            // =====??????
    Encoder_Init_TIM3();            // =====??????

    PWM_Init(7199, 9);              // =====?????? ??1000

    colorful_led_Init();            // LED for NFC card-feedback

    SysTick_Config(72000000/1000);  // ?????,?1ms??????

    printf("QST??\r\n");

    /**????**/
    while(1)
    {
        NFC_Handler();              // NFC poll: wakeup / search card
        delay_ms(100);
    }
}
