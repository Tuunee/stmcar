/*
 * 14.0_BleCar —— 蓝牙遥控小车 (Hi3861 端)
 *
 * 功能: 手机蓝牙调试器发单字符指令, Hi3861 解析后通过 UART2
 *       发 6 字节协议帧给 STM32 控制电机
 *
 * 硬件连接:
 *   蓝牙模块 (HC-05/JDY-31 等):
 *     Hi3861 GPIO0  = UART1_TXD ──→ 蓝牙模块 RXD
 *     Hi3861 GPIO1  = UART1_RXD ←── 蓝牙模块 TXD
 *     波特率 9600 (与 5.0_Uart_BLE 作业一致)
 *
 *   STM32F103 (电机控制从机):
 *     Hi3861 GPIO11 = UART2_TXD ──→ STM32 PA10 (USART1 RX)
 *     Hi3861 GPIO12 = UART2_RXD ←── STM32 PA9  (USART1 TX)
 *     波特率 115200
 *
 * 蓝牙指令 (大小写均可, 自动过滤 \r \n):
 *   W = 前进 (速度70)     S = 后退 (速度70)
 *   A = 左转 (原地)       D = 右转 (原地)
 *   O = 停止
 *   I = 前进 (速度100)    K = 前进 (速度150)
 *
 * 协议帧 (官方 12.0 任务书 6 字节帧):
 *   0xFC | 左方向(0正/1反) | 左速度(0~150) | 右方向 | 右速度 | 0xFD
 *
 * 安全机制:
 *   1. 上电后先发一帧停止指令, 防止 STM32 残留旧速度
 *   2. 蓝牙 600ms 收不到数据时, 重发当前指令 (给 STM32 保活)
 *      STM32 侧 1.5s 收不到帧会自动停车 (需配套新 control_system.c)
 */
#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_errno.h"
#include "hi_uart.h"

/* ==================== 配置宏 ==================== */
#define BLE_UART_IDX      HI_UART_IDX_1     /* 蓝牙模块: GPIO0/1, 9600 */
#define CAR_UART_IDX      HI_UART_IDX_2     /* STM32:   GPIO11/12, 115200 */

/* 速度档位 (协议范围 0~150) */
#define SPD_W             70    /* W 前进默认速度 */
#define SPD_I             100   /* I 中速前进 */
#define SPD_K             150   /* K 全速前进 */
#define SPD_S             70    /* S 后退速度 */
#define SPD_TURN          60    /* A/D 原地转弯速度 */

/* 右轮偏紧补偿: 右轮速度按百分比加大, 抵消机械阻力差异
 * 直行时向右偏 → 调大; 向左偏 → 调小或改负值给左轮补偿 */
#define RIGHT_COMP_PCT    15    /* 右轮 +15% */

/* 诊断打印开关: 1=打印收到的每个字节和心跳; 0=静默 */
#define BLE_DEBUG         0

/* 蓝牙读超时: 超时后向 STM32 重发当前指令 (保活) */
#define BLE_TIMEOUT_MS    600

#define TASK_STACK_SIZE   (1024 * 4)
#define TASK_PRIO         25

/* ==================== 全局变量 ==================== */
static uint8_t txbuf[6];
static int cur_l = 0, cur_r = 0;   /* 当前运动指令 (带符号) */

/* ==================== 发送 6 字节协议帧给 STM32 ==================== */
static void SendCar(int motorL, int motorR)
{
    uint8_t l_dir = 0, r_dir = 0;

    if (motorL < 0) { l_dir = 1; motorL = -motorL; }
    if (motorR < 0) { r_dir = 1; motorR = -motorR; }

    /* 右轮偏紧补偿: 右轮目标速度按百分比加大 */
    motorR = motorR * (100 + RIGHT_COMP_PCT) / 100;

    if (motorL > 150) motorL = 150;
    if (motorR > 150) motorR = 150;

    txbuf[0] = 0xFC;
    txbuf[1] = l_dir;
    txbuf[2] = (uint8_t)motorL;
    txbuf[3] = r_dir;
    txbuf[4] = (uint8_t)motorR;
    txbuf[5] = 0xFD;
    UartWrite(CAR_UART_IDX, txbuf, 6);
}

/* 执行指令并打印 (只在指令变化时打印) */
static void ExecCmd(int l, int r, const char *name)
{
    if (l != cur_l || r != cur_r) {
        printf("[BleCar] %s (L=%+d R=%+d)\r\n", name, l, r);
        cur_l = l;
        cur_r = r;
    }
    SendCar(l, r);
}

/* ==================== 主任务 ==================== */
static void BleCarTask(void)
{
    uint8_t ch;
    hi_s32 n;
    uint32_t loop_cnt = 0;

    printf("[BleCar] 蓝牙遥控任务启动 (右轮补偿+%d%%)\r\n", RIGHT_COMP_PCT);
    printf("[BleCar] 指令: W前进 A左转 D右转 S后退 O停止 I前进100 K前进150\r\n");

    /* 上电先停车, 防止 STM32 残留旧速度 */
    cur_l = 0;
    cur_r = 0;
    SendCar(0, 0);

    while (1) {
        /* 带超时读蓝牙 (600ms), 超时返回 0 */
        n = hi_uart_read_timeout(BLE_UART_IDX, &ch, 1, BLE_TIMEOUT_MS);
        loop_cnt++;

        if (n < 0) {
            /* 读出错: 防止空转打满 CPU */
            osDelay(50);
            continue;
        }

        if (n == 0) {
            /* 超时: 重发当前指令, 给 STM32 保活
             * (STM32 侧 1.5s 收不到帧会自动停车) */
            SendCar(cur_l, cur_r);
#if BLE_DEBUG
            if (loop_cnt % 10 == 1) {
                printf("[BleCar][心跳] loop=%u\r\n", (unsigned)loop_cnt);
            }
#endif
            continue;
        }

#if BLE_DEBUG
        printf("[BleCar][收] 0x%02X\r\n", ch);
#endif

        /* 过滤回车换行, 统一转大写 */
        if (ch == '\r' || ch == '\n' || ch == 0) continue;
        if (ch >= 'a' && ch <= 'z') ch -= 32;

        switch (ch) {
        case 'W': ExecCmd( SPD_W,  SPD_W, "前进");  break;
        case 'I': ExecCmd( SPD_I,  SPD_I, "中速前进"); break;
        case 'K': ExecCmd( SPD_K,  SPD_K, "全速前进"); break;
        case 'S': ExecCmd(-SPD_S, -SPD_S, "后退");  break;
        case 'A': ExecCmd(-SPD_TURN, SPD_TURN, "左转"); break;
        case 'D': ExecCmd( SPD_TURN,-SPD_TURN, "右转"); break;
        case 'O': ExecCmd(0, 0, "停止");            break;
        default:
            /* 未知字符: 忽略, 不改变当前运动状态 */
            break;
        }
    }
}

/* ==================== 入口 ==================== */
static void BleCarEntry(void)
{
    uint32_t ret;

    GpioInit();

    /* UART1: 蓝牙模块 GPIO0=TX GPIO1=RX, 9600-8N1 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
    WifiIotUartAttribute ble_attr = {
        .baudRate = 9600,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = UartInit(BLE_UART_IDX, &ble_attr, NULL);
    if (ret != WIFI_IOT_SUCCESS) {
        printf("[BleCar] 蓝牙串口(UART1)初始化失败! err=0x%x\r\n", ret);
        return;
    }
    printf("[BleCar] 蓝牙串口(UART1)初始化成功\r\n");

    /* UART2: STM32 GPIO11=TX GPIO12=RX, 115200-8N1 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    WifiIotUartAttribute car_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = UartInit(CAR_UART_IDX, &car_attr, NULL);
    if (ret != WIFI_IOT_SUCCESS) {
        printf("[BleCar] STM32串口(UART2)初始化失败! err=0x%x\r\n", ret);
        printf("[BleCar] 请检查 usr_config.mk 是否已开启 CONFIG_UART2_SUPPORT=y\r\n");
        return;
    }
    printf("[BleCar] STM32串口(UART2)初始化成功\r\n");

    /* 创建主任务 */
    osThreadAttr_t attr = {0};
    attr.name       = "BleCarTask";
    attr.attr_bits  = 0U;
    attr.cb_mem     = NULL;
    attr.cb_size    = 0U;
    attr.stack_mem  = NULL;
    attr.stack_size = TASK_STACK_SIZE;
    attr.priority   = TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)BleCarTask, NULL, &attr) == NULL) {
        printf("[BleCar] 创建任务失败!\r\n");
    }
}

APP_FEATURE_INIT(BleCarEntry);
