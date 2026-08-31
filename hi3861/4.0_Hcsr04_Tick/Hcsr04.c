#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"
//HC-SR04 超声波测距模块通过GPIO7和8连接到3861
#define GPIO_8  8
#define GPIO_7  7
#define GPIO_FUNC  0
//测距功能实现
float GetDistance (void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    hi_io_set_func(GPIO_8, GPIO_FUNC);

    GpioSetDir(GPIO_8, WIFI_IOT_GPIO_DIR_IN);//GPIO_8设置为输入引脚
    GpioSetDir(GPIO_7, WIFI_IOT_GPIO_DIR_OUT);//GPIO_7设置为输出引脚

    //GPIO_7输出一个脉冲触发信号到超声波测距模块 至少10us
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);

    //超声波测距模块接收到GPIO_7输出的脉冲触发信号后,模块输出回响信号(高电平)到GPIO_8
    while (1) {
        GpioGetInputVal(GPIO_8, &value);

        //测量回响信号(高电平)时间
        if ( value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
    }
    //距离=高电平时间*0.034 / 2
    distance = time * 0.034 / 2;
    return distance;
}

/*定时器1回调: 每3s测量一次距离*/
void Timer1Callback(void *arg)
{
    (void)arg;
    float distance = GetDistance();
    printf("distance is %.1f (cm)\r\n", distance);
}

/*定时器2回调: 打印当前tick值*/
void Timer2Callback(void *arg)
{
    (void)arg;
    //osKernelGetTickCount()返回系统启动以来的tick数, 1 tick = 10ms
    printf("current tick is %u\r\n", (unsigned int)osKernelGetTickCount());
}

/*任务入口*/
static void Hcsr04(void)
{
    WatchDogDisable();   //关闭看门狗

    //创建两个周期型软件定时器
    osTimerId_t timer1 = osTimerNew(Timer1Callback, osTimerPeriodic, NULL, NULL);
    osTimerId_t timer2 = osTimerNew(Timer2Callback, osTimerPeriodic, NULL, NULL);

    if (timer1 == NULL) {
        printf("Falied to create timer1!\n");
    } else {
        osTimerStart(timer1, 300U);//300个tick = 3s (系统tick为100Hz, 1 tick = 10ms)
    }

    if (timer2 == NULL) {
        printf("Falied to create timer2!\n");
    } else {
        osTimerStart(timer2, 100U);//100个tick = 1s
    }
}

APP_FEATURE_INIT(Hcsr04);//任务启动
