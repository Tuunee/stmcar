#include "hal_bsp_ap3216c.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"
#include <unistd.h>
#include <stdio.h>

#define AP3216C_SYSTEM_ADDR     0x00
#define AP3216C_IR_L_ADDR       0x0A
#define AP3216C_IR_H_ADDR       0x0B
#define AP3216C_ALS_L_ADDR      0x0C
#define AP3216C_ALS_H_ADDR      0x0D
#define AP3216C_PS_L_ADDR       0x0E
#define AP3216C_PS_H_ADDR       0x0F

/* ALS 范围系数: reg 0x10 bits[5:4]
 * 00 -> 0.2 lux/count (800 lux max)
 * 01 -> 1 lux/count (1600 lux max)
 * 10 -> 32/32 = 1? No... datasheet says coef=32 for 5840 lux
 * Actually lux = raw / coefficient, coefficient depends on range
 */
#define ALS_COEFFICIENT  32   // 5840 lux range: lux = raw / 32

static uint32_t AP3216C_WriteByte(uint8_t byte)
{
    WifiIotI2cData i2cData = {0};
    i2cData.sendBuf = &byte;
    i2cData.sendLen = 1;
    return I2cWrite(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &i2cData);
}

static uint32_t AP3216C_RecvData(uint8_t *data, size_t size)
{
    WifiIotI2cData i2cData = {0};
    i2cData.receiveBuf = data;
    i2cData.receiveLen = size;
    return I2cRead(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &i2cData);
}

uint32_t AP3216C_WriteReg(uint8_t regAddr, uint8_t byte)
{
    uint8_t buffer[] = {regAddr, byte};
    WifiIotI2cData i2cData = {0};
    i2cData.sendBuf = buffer;
    i2cData.sendLen = 2;
    return I2cWrite(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &i2cData);
}

uint32_t AP3216C_ReadReg(uint8_t regAddr, uint8_t *byte)
{
    uint32_t result = 0;
    result = AP3216C_WriteByte(regAddr);
    if (result != 0) return result;
    result = AP3216C_RecvData(byte, 1);
    if (result != 0) return result;
    return 0;
}

uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData)
{
    uint8_t ir_l = 0, ir_h = 0, als_l = 0, als_h = 0, ps_l = 0, ps_h = 0;
    uint32_t als_raw;

    AP3216C_ReadReg(AP3216C_IR_L_ADDR, &ir_l);
    AP3216C_ReadReg(AP3216C_IR_H_ADDR, &ir_h);
    AP3216C_ReadReg(AP3216C_ALS_L_ADDR, &als_l);
    AP3216C_ReadReg(AP3216C_ALS_H_ADDR, &als_h);
    AP3216C_ReadReg(AP3216C_PS_L_ADDR, &ps_l);
    AP3216C_ReadReg(AP3216C_PS_H_ADDR, &ps_h);

    // IR: 10-bit, IR_OF=bit7 of IR_L
    if (ir_l & 0x80)
        *irData = 0;
    else
        *irData = ((uint16_t)ir_h << 2) | (ir_l & 0x03);

    // ALS: 16-bit raw -> lux = raw / coefficient
    als_raw = ((uint16_t)als_h << 8) | als_l;
    *alsData = (uint16_t)(als_raw / ALS_COEFFICIENT);

    // PS: 10-bit in bits[15:6] of combined PS_H:PS_L, PS_OF=bit6 of PS_L
    if (ps_l & 0x40)
        *psData = 0;
    else
        *psData = ((uint16_t)ps_h << 8 | ps_l) >> 6;

    return 0;
}

uint32_t AP3216C_Init(void)
{
    uint32_t result;
    uint8_t val;
    int retry;
    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);

    I2cInit(WIFI_IOT_I2C_IDX_0, 400000);
    usleep(200 * 1000);

    result = AP3216C_WriteReg(AP3216C_SYSTEM_ADDR, 0x07);
    if (result != 0) {
        uint8_t probe = 0x00;
        unsigned short a;
        int found = 0;
        printf("[AP3216C] I2C fail, scanning bus...\n");
        for (a = 1; a < 120; a++) {
            WifiIotI2cData pd = {&probe, 1, NULL, 0};
            if (I2cWrite(WIFI_IOT_I2C_IDX_0, a << 1, &pd) == 0) {
                printf("[AP3216C] found device, 7-bit addr = 0x%02X\n", a);
                found++;
            }
        }
        if (found == 0) {
            printf("[AP3216C] no device! SDA->GPIO10, SCL->GPIO9, VCC->3.3V, GND\n");
        }
        return result;
    }

    retry = 0;
    do {
        usleep(50 * 1000);
        AP3216C_ReadReg(AP3216C_SYSTEM_ADDR, &val);
        retry++;
        if (retry > 20) return 1;
    } while (val != 0x00);

    AP3216C_WriteReg(0x10, 0x20);  // ALS range: 5840 lux (coef=32)
    AP3216C_WriteReg(0x20, 0x03);  // PS config
    usleep(10 * 1000);

    // 直接用单次测量模式(mode=3), 连续模式在这颗芯片上不稳定
    AP3216C_WriteReg(AP3216C_SYSTEM_ADDR, 0x03);
    usleep(10 * 1000);

    usleep(300 * 1000);
    return 0;
}
