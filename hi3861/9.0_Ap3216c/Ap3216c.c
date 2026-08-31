#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ap3216c.h"

static void sensor_task(void)
{
    uint16_t ir, als, ps;
    usleep(500 * 1000);

    while (1)
    {
        // 单次测量模式: 触发一次转换
        AP3216C_WriteReg(0x00, 0x03);
        usleep(200 * 1000);  // 等ALS+PS+IR全部转换完成

        AP3216C_ReadData(&ir, &als, &ps);
        printf("IR=%u ALS=%u PS=%u\n", ir, als, ps);

        usleep(1000 * 1000);
    }
}

static void ap3216c_demo(void)
{
    if (AP3216C_Init() != 0) {
        return;
    }

    osThreadAttr_t attr;
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 2;
    attr.name = "ap3216c_sensor";
    attr.priority = osPriorityNormal;

    if (osThreadNew((osThreadFunc_t)sensor_task, NULL, &attr) == NULL) {
        printf("[AP3216C] Failed to create sensor_task!\n");
    }
}

APP_FEATURE_INIT(ap3216c_demo);
