#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"

osMutexId_t mutex_id;
#define GPIO2 2
static volatile uint8_t flag = 0;    //舵机旋转角度标志位

static void thread1(void);
static void thread2(void);
static void thread3(void);

//查阅小车原理图可知，SG90舵机通过GPIO2与3861连接
//SG90舵机的控制需要MCU产生一个周期为20ms的脉冲信号，以0.5ms到2.5ms的高电平来控制舵机转动的角度
//输出20000微秒的脉冲信号(x微秒高电平，20000‑x微秒低电平)
void set_angle( unsigned int duty) {
    GpioSetDir(GPIO2, WIFI_IOT_GPIO_DIR_OUT);//设置GPIO2为输出模式

    //GPIO2输出x微秒高电平
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);

    //GPIO2输出20000‑x微秒低电平
    GpioSetOutputVal(GPIO2, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/*
1、依据角度与脉冲的关系，设置高电平时间为500微秒，控制舵机旋转0度。
2、发送10次脉冲信号。
*/
void engine_run_0(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(500);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为1000微秒，控制舵机旋转45度。
2、发送10次脉冲信号。
*/
void engine_run_45(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(1000);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为1500微秒，控制舵机旋转90度。
2、发送10次脉冲信号。
*/
void engine_run_90(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(1500);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为2000微秒，控制舵机旋转135度。
2、发送10次脉冲信号。
*/
void engine_run_135(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(2000);
    }
}

/*
1、依据角度与脉冲的关系，设置高电平时间为2500微秒，控制舵机向右旋转180度。
2、发送10次脉冲信号。
*/
void engine_run_180(void)
{
    for (int i = 0; i <10; i++)
    {
        set_angle(2500);
    }
}


/*****任务创建*****/
static void SG90(void)
{
    GpioInit();//初始化GPIO
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2,WIFI_IOT_IO_FUNC_GPIO_2_GPIO);   //设置GPIO模式
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_2,WIFI_IOT_GPIO_DIR_OUT);        //设置为输出模式

    osThreadAttr_t  attr;
    attr.attr_bits  = 0U;
    attr.cb_mem  = NULL;
    attr.cb_size  = 0U;
    attr.stack_mem  = NULL;
    attr.stack_size  = 1024 * 4;
    mutex_id = osMutexNew(NULL);//创建互斥锁
    if (mutex_id == NULL)
    {
        printf("Falied to create Mutex!\n");
        return;
    }

    attr.name  = "thread1";
    attr.priority  = 26;
    if (osThreadNew((osThreadFunc_t) thread1, NULL, &attr) == NULL)
    {
        printf("Falied to create thread1!\n");
    }
    attr.name  = "thread2";
    attr.priority  = 25;
    if (osThreadNew((osThreadFunc_t) thread2, NULL, &attr) == NULL)
    {
        printf("Falied to create thread2!\n");
    }
    attr.name  = "thread3";
    attr.priority  = 24;
    if (osThreadNew((osThreadFunc_t) thread3, NULL, &attr) == NULL)
    {
        printf("Falied to create thread3!\n");
    }
}

/*****任务一*****/
static void thread1(void)
{
//延时1秒中
osDelay(100U);
  while (1)
  {
//获取互斥锁
    osMutexAcquire(mutex_id, osWaitForever);
    printf("thread1 is runing.\r\n");
    flag=90;      //设置标志位  90度
    engine_run_90();     //舵机转90度
//持有互斥锁5秒，其他任务无法获取
    osDelay(500U);
//释放互斥锁
    osMutexRelease(mutex_id);
  }
}

static void thread2(void)
{
  //延时1秒中
  osDelay(100U);
  //中优先级没有获取互斥锁，互斥锁对它没有影响
  while (1)
  {
    printf("thread2 is runing.\r\n");
    switch(flag)
    {
    case 90 :
        printf("SG90 turn 90 du.\r\n");break;
    case 180 :
        printf("SG90 turn 180 du.\r\n");break;
    default :
        break;
    }

    flag=0;    //清除标志位
    osDelay(100);
  }
}

static void thread3(void)
{
  while (1)
  {
//先获取互斥锁
    osMutexAcquire(mutex_id, osWaitForever);
    printf("thread3 is runing.\r\n");
    flag=180;      //设置标志位  180度
    engine_run_180();     //舵机转180度
//低优先级持有互斥锁3秒，任务1无法执行
    osDelay(300U);
//释放互斥锁
    osMutexRelease(mutex_id);
  }
}

APP_FEATURE_INIT(SG90); ///任务启动
