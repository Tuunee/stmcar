/*
 * 15.0_FenceAvoid —— 围场防撞小车 (Hi3861 端)
 *
 * 功能: 小车在围场里直线前进, HC-SR04 超声波检测到围栏立即停止
 *       (不碰撞), 后退一段距离, 随机左转或右向后继续直行, 循环往复
 *
 * 硬件连接:
 *   HC-SR04 超声波模块 (朝前安装, 与 4.0_Hcsr04_Tick 作业一致):
 *     Hi3861 GPIO7 = TRIG (触发脚)
 *     Hi3861 GPIO8 = ECHO (回响脚)
 *
 *   STM32F103 (电机控制从机, 与 14.0_BleCar 相同):
 *     Hi3861 GPIO11 = UART2_TXD ──→ STM32 PA10 (USART1 RX)
 *     Hi3861 GPIO12 = UART2_RXD ←── STM32 PA9  (USART1 TX)
 *     波特率 115200
 *
 *   (蓝牙 UART1 未使用, 本程序为自主模式)
 *
 * 协议帧 (官方 12.0 任务书 6 字节帧):
 *   0xFC | 左方向(0正/1反) | 左速度(0~150) | 右方向 | 右速度 | 0xFD
 *
 * 状态机:
 *   FORWARD ──(超声距离 < STOP_CM 连续2次)──→ STOP
 *   STOP    ──(停稳 300ms)──────────────────→ BACK
 *   BACK    ──(后退 1.5s)───────────────────→ TURN
 *   TURN    ──(随机左/右转 1.5s)────────────→ FORWARD
 *
 * 保活机制: 每 500ms 重发当前指令帧
 *          (STM32 v3 固件 2.5s 收不到帧会自动停车)
 */
#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "wifiiot_errno.h"
#include "hi_uart.h"
#include "hi_io.h"
#include "hi_time.h"

/* ==================== 配置宏 ==================== */
#define CAR_UART_IDX      HI_UART_IDX_2     /* STM32: GPIO11/12, 115200 */

/* 超声波引脚 (与 4.0_Hcsr04_Tick 作业一致) */
#define US_TRIG_GPIO      7
#define US_ECHO_GPIO      8
#define US_IO_FUNC        0                 /* 普通GPIO功能 */

/* 停车距离: 小于该值判定"即将撞围栏" (cm) */
#define STOP_CM           10
/* 连续 N 次低于 STOP_CM 才停车, 防误判 */
#define HIT_DEBOUNCE      2

/* 速度档位 (协议范围 0~150) */
#define SPD_FWD           70    /* 前进速度 */
#define SPD_BACK          70    /* 后退速度 */
#define SPD_TURN          70    /* 原地转弯速度 */

/* 右轮偏紧硬件补偿: 右轮速度放大百分比 */
#define RIGHT_COMP_PCT    15

/* 时长 (单位=拍, 1拍=100ms) */
#define STOP_TICKS        3     /* 停稳 300ms */
#define BACK_TICKS        15    /* 后退 1.5s */
#define TURN_TICKS        50    /* 转弯 5s */
#define KEEPALIVE_TICK    5     /* 每 5 拍(500ms) 重发指令帧 */

/* 超声波回响等待超时 (us), 超时视为无障碍物 */
#define ECHO_TIMEOUT_US   30000

#define TASK_STACK_SIZE   (1024 * 4)
#define TASK_PRIO         25

/* ==================== 状态机 ==================== */
enum State {
    ST_STOP = 0,     /* 上电默认停止 */
    ST_FORWARD,
    ST_BRAKE,        /* 检测到围栏, 停稳 */
    ST_BACK,
    ST_TURN,
};

/* ==================== 全局变量 ==================== */
static uint8_t txbuf[6];
static enum State cur_state = ST_STOP;
static int state_tick = 0;

/* ==================== 超声波测距 (cm) ==================== */
static float GetDistance(void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    unsigned long wait_start;

    hi_io_set_func(US_ECHO_GPIO, US_IO_FUNC);
    GpioSetDir(US_ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);   /* ECHO 输入 */
    GpioSetDir(US_TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);  /* TRIG 输出 */

    /* TRIG 输出至少 10us 高电平触发 */
    GpioSetOutputVal(US_TRIG_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(US_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);

    /* 测量 ECHO 高电平时间 (带超时保护, 模块没回响不会卡死) */
    wait_start = hi_get_us();
    while (1) {
        GpioGetInputVal(US_ECHO_GPIO, &value);

        if (value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
        /* 超时: 无回响, 当作无障碍 */
        if (flag == 0 && (hi_get_us() - wait_start) > ECHO_TIMEOUT_US) {
            return 999.0f;
        }
        if (flag == 1 && (hi_get_us() - start_time) > ECHO_TIMEOUT_US) {
            return 999.0f;
        }
    }

    /* 距离(cm) = 高电平时间(us) × 0.034 / 2 */
    distance = time * 0.034 / 2;
    return distance;
}

/* ==================== 发送 6 字节协议帧给 STM32 ==================== */
static void SendCar(int motorL, int motorR)
{
    uint8_t l_dir = 0, r_dir = 0;

    if (motorL < 0) { l_dir = 1; motorL = -motorL; }
    if (motorR < 0) { r_dir = 1; motorR = -motorR; }

    /* 右轮偏紧: 右轮速度放大补偿, 修正直行右偏 */
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

/* 前进(保持当前指令并保活重发时使用) */
static void CarForward(void)  { SendCar( SPD_FWD,  SPD_FWD); }
static void CarBackward(void) { SendCar(-SPD_BACK, -SPD_BACK); }
static void CarStop(void)     { SendCar(0, 0); }
static void CarTurnLeft(void) { SendCar(-SPD_TURN, SPD_TURN); }
static void CarTurnRight(void){ SendCar( SPD_TURN,-SPD_TURN); }

/* ==================== 主任务 ==================== */
static void FenceAvoidTask(void)
{
    float dist;
    int hit_cnt = 0;
    int turn_left = 0;

    printf("[FenceAvoid] 任务启动, 上电即开始前进\r\n");
    printf("[FenceAvoid] 停车距离 STOP_CM=%dcm, 前进=%d 后退=%d 转弯=%d\r\n",
           STOP_CM, SPD_FWD, SPD_BACK, SPD_TURN);

    cur_state = ST_FORWARD;
    state_tick = 0;
    CarForward();

    while (1) {
        /* 每 100ms 一拍 (osDelay 参数是 tick, 1 tick = 10ms, 故传 10) */
        osDelay(10);
        state_tick++;

        /* 保活: 每 5 拍(500ms)重发当前指令帧, 防 STM32 失联停车
         * 注意: ST_TURN 不在此列——转弯指令由状态机每拍重发,
         *       若在这里发停车帧会把转弯打断成"罚站" */
        if (state_tick % KEEPALIVE_TICK == 0) {
            switch (cur_state) {
            case ST_FORWARD: CarForward();   break;
            case ST_BRAKE:   CarStop();      break;
            case ST_BACK:    CarBackward();  break;
            case ST_STOP:    CarStop();      break;
            default:                          /* ST_TURN: 不处理 */ break;
            }
        }

        switch (cur_state) {
        /* ---- 直行 + 超声检测 ---- */
        case ST_FORWARD:
            dist = GetDistance();
            if (dist < STOP_CM) {
                hit_cnt++;
                printf("[FenceAvoid] dist=%.1fcm (第%d次)\r\n", dist, hit_cnt);
                if (hit_cnt >= HIT_DEBOUNCE) {
                    printf("[FenceAvoid] 围栏! 刹车\r\n");
                    cur_state = ST_BRAKE;
                    state_tick = 0;
                    hit_cnt = 0;
                    CarStop();
                    break;
                }
            } else {
                hit_cnt = 0;
            }
            break;

        /* ---- 刹车停稳 ---- */
        case ST_BRAKE:
            if (state_tick >= STOP_TICKS) {
                printf("[FenceAvoid] 已停稳, 后退\r\n");
                cur_state = ST_BACK;
                state_tick = 0;
                CarBackward();
            }
            break;

        /* ---- 后退 ---- */
        case ST_BACK:
            CarBackward();   /* 每拍重发, 保证期间指令不断 */
            if (state_tick >= BACK_TICKS) {
                /* 用系统 tick 奇偶性随机选转向方向 */
                turn_left = (int)(osKernelGetTickCount() & 1);
                printf("[FenceAvoid] 后退完成, %s\r\n", turn_left ? "左转" : "右转");
                cur_state = ST_TURN;
                state_tick = 0;
                if (turn_left) CarTurnLeft(); else CarTurnRight();
            }
            break;

        /* ---- 原地转弯: 每拍重发转弯指令, 否则会被保活停车帧打断 ---- */
        case ST_TURN:
            if (turn_left) CarTurnLeft(); else CarTurnRight();
            if (state_tick >= TURN_TICKS) {
                printf("[FenceAvoid] 转弯完成, 继续直行\r\n");
                cur_state = ST_FORWARD;
                state_tick = 0;
                hit_cnt = 0;
                CarForward();
            }
            break;

        default:
            CarStop();
            cur_state = ST_FORWARD;
            state_tick = 0;
            break;
        }
    }
}

/* ==================== 入口 ==================== */
static void FenceAvoidEntry(void)
{
    uint32_t ret;

    GpioInit();

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
        printf("[FenceAvoid] STM32串口(UART2)初始化失败! err=0x%x\r\n", ret);
        printf("[FenceAvoid] 请检查 usr_config.mk 是否已开启 CONFIG_UART2_SUPPORT=y\r\n");
        return;
    }
    printf("[FenceAvoid] STM32串口(UART2)初始化成功\r\n");

    /* 超声波引脚: GPIO7=TRIG 输出, GPIO8=ECHO 输入 */
    hi_io_set_func(US_TRIG_GPIO, US_IO_FUNC);
    GpioSetDir(US_TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(US_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);

    hi_io_set_func(US_ECHO_GPIO, US_IO_FUNC);
    GpioSetDir(US_ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);

    /* 创建主任务 */
    osThreadAttr_t attr = {0};
    attr.name       = "FenceAvoidTask";
    attr.attr_bits  = 0U;
    attr.cb_mem     = NULL;
    attr.cb_size    = 0U;
    attr.stack_mem  = NULL;
    attr.stack_size = TASK_STACK_SIZE;
    attr.priority   = TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)FenceAvoidTask, NULL, &attr) == NULL) {
        printf("[FenceAvoid] 创建任务失败!\r\n");
    }
}

APP_FEATURE_INIT(FenceAvoidEntry);
