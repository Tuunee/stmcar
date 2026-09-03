/*
 * 16.0_TapeAvoid —— 黑胶带围场防出界小车 (Hi3861 端)
 *
 * 功能: 黑色胶带围成一个围场, 小车在围场内直线前进;
 *       红外对管检测到黑线 (围场边界) 立即停止, 不能压到黑线;
 *       后退一段距离, 随机左转或右转后继续直行, 循环往复
 *
 * 硬件连接:
 *   TCRT5000 红外对管 ×2 (朝下安装, 与 2.0_TCRT_Timer 作业一致):
 *     Hi3861 GPIO13 = 左红外输出 (低电平=识别到黑色, 高电平=白色)
 *     Hi3861 GPIO14 = 右红外输出 (同上)
 *
 *   STM32F103 (电机控制从机, 与 15.0_FenceAvoid 相同):
 *     Hi3861 GPIO11 = UART2_TXD ──→ STM32 PA10 (USART1 RX)
 *     Hi3861 GPIO12 = UART2_RXD ←── STM32 PA9  (USART1 TX)
 *     波特率 115200
 *
 * 协议帧 (官方 12.0 任务书 6 字节帧):
 *   0xFC | 左方向(0正/1反) | 左速度(0~150) | 右方向 | 右速度 | 0xFD
 *
 * 状态机:
 *   FORWARD ──(任一侧红外识别到黑线, 连续2拍)──→ BRAKE
 *   BRAKE   ──(停稳 300ms)─────────────────────→ BACK
 *   BACK    ──(后退 1.5s)──────────────────────→ TURN
 *   TURN    ──(随机左/右转 1.5s)───────────────→ FORWARD
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

/* ==================== 配置宏 ==================== */
#define CAR_UART_IDX      HI_UART_IDX_2     /* STM32: GPIO11/12, 115200 */

/* 红外对管引脚 (与 2.0_TCRT_Timer 作业一致, 低电平=黑线) */
#define IR_LEFT_GPIO      13
#define IR_RIGHT_GPIO     14

/* 速度档位 (协议范围 0~150) */
#define SPD_FWD           70    /* 前进速度 */
#define SPD_BACK          70    /* 后退速度 */
#define SPD_TURN          70    /* 原地转弯速度 */

/* 右轮偏紧硬件补偿: 右轮速度放大百分比 */
#define RIGHT_COMP_PCT    15

/* 时长 (单位=拍, 1拍=100ms) */
#define STOP_TICKS        3     /* 刹车停稳 300ms */
#define BACK_TICKS        15    /* 后退 1.5s */
#define TURN_TICKS        50    /* 转弯 5s */
#define KEEPALIVE_TICK    5     /* 每 5 拍(500ms) 重发指令帧 */

/* 连续 N 拍识别到黑线才刹车, 防误判 */
#define LINE_DEBOUNCE     2

#define TASK_STACK_SIZE   (1024 * 4)
#define TASK_PRIO         25

/* ==================== 状态机 ==================== */
enum State {
    ST_STOP = 0,     /* 上电默认停止 */
    ST_FORWARD,
    ST_BRAKE,        /* 检测到黑线, 刹车 */
    ST_BACK,
    ST_TURN,
};

/* ==================== 全局变量 ==================== */
static uint8_t txbuf[6];
static enum State cur_state = ST_STOP;
static int state_tick = 0;

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

static void CarForward(void)   { SendCar( SPD_FWD,  SPD_FWD); }
static void CarBackward(void)  { SendCar(-SPD_BACK, -SPD_BACK); }
static void CarStop(void)      { SendCar(0, 0); }
static void CarTurnLeft(void)  { SendCar(-SPD_TURN, SPD_TURN); }
static void CarTurnRight(void) { SendCar( SPD_TURN,-SPD_TURN); }

/* ==================== 红外读取 ==================== */
/* 任一侧识别到黑线返回1
 * 实测本板极性: 高电平(1)=黑线, 低电平(0)=白色地面
 * (2026-09-03 实测: 白桌 L=0/R=0, 黑桌 L=1/R=0) */
static int IsLineDetected(void)
{
    WifiIotGpioValue lv, rv;

    GpioGetInputVal(IR_LEFT_GPIO, &lv);
    GpioGetInputVal(IR_RIGHT_GPIO, &rv);

    if (lv == WIFI_IOT_GPIO_VALUE1 || rv == WIFI_IOT_GPIO_VALUE1) {
        return 1;
    }
    return 0;
}

/* ==================== 主任务 ==================== */
static void TapeAvoidTask(void)
{
    int line_cnt = 0;
    int turn_left = 0;
    WifiIotGpioValue lv, rv;

    printf("[TapeAvoid] 任务启动, 上电即开始前进\r\n");
    printf("[TapeAvoid] 前进=%d 后退=%d 转弯=%d\r\n",
           SPD_FWD, SPD_BACK, SPD_TURN);

    /* ---- 传感器原始值诊断: 开机先采 3 秒不动 ----
     * 本板实测极性: 1=黑(黑线), 0=白(正常地面)
     * 放白桌上应稳定打印 L=0 R=0; 若打印 L=1 或 R=1,
     * 说明该传感器在白地面上也报黑, 需检查安装高度/表面材质 */
    for (int i = 0; i < 30; i++) {
        GpioGetInputVal(IR_LEFT_GPIO, &lv);
        GpioGetInputVal(IR_RIGHT_GPIO, &rv);
        printf("[TapeAvoid] 原始值 L=%d R=%d (1=黑 0=白)\r\n", (int)lv, (int)rv);
        osDelay(10);
    }
    printf("[TapeAvoid] 诊断结束, 开始运行\r\n");

    cur_state = ST_FORWARD;
    state_tick = 0;
    line_cnt = 0;
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
        /* ---- 直行 + 黑线检测 ---- */
        case ST_FORWARD:
            if (IsLineDetected()) {
                line_cnt++;
                GpioGetInputVal(IR_LEFT_GPIO, &lv);
                GpioGetInputVal(IR_RIGHT_GPIO, &rv);
                printf("[TapeAvoid] 黑线! L=%d R=%d (第%d拍)\r\n",
                       (int)lv, (int)rv, line_cnt);
                if (line_cnt >= LINE_DEBOUNCE) {
                    printf("[TapeAvoid] 到达边界, 刹车\r\n");
                    cur_state = ST_BRAKE;
                    state_tick = 0;
                    line_cnt = 0;
                    CarStop();
                    break;
                }
            } else {
                line_cnt = 0;
            }
            break;

        /* ---- 刹车停稳 ---- */
        case ST_BRAKE:
            if (state_tick >= STOP_TICKS) {
                printf("[TapeAvoid] 已停稳, 后退\r\n");
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
                printf("[TapeAvoid] 后退完成, %s\r\n", turn_left ? "左转" : "右转");
                cur_state = ST_TURN;
                state_tick = 0;
                if (turn_left) CarTurnLeft(); else CarTurnRight();
            }
            break;

        /* ---- 原地转弯: 每拍重发转弯指令, 否则会被保活停车帧打断 ---- */
        case ST_TURN:
            if (turn_left) CarTurnLeft(); else CarTurnRight();
            if (state_tick >= TURN_TICKS) {
                printf("[TapeAvoid] 转弯完成, 继续直行\r\n");
                cur_state = ST_FORWARD;
                state_tick = 0;
                line_cnt = 0;
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
static void TapeAvoidEntry(void)
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
        printf("[TapeAvoid] STM32串口(UART2)初始化失败! err=0x%x\r\n", ret);
        printf("[TapeAvoid] 请检查 usr_config.mk 是否已开启 CONFIG_UART2_SUPPORT=y\r\n");
        return;
    }
    printf("[TapeAvoid] STM32串口(UART2)初始化成功\r\n");

    /* 红外对管 GPIO13/14: 普通 GPIO 输入 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_GPIO_DIR_IN);

    /* 创建主任务 */
    osThreadAttr_t attr = {0};
    attr.name       = "TapeAvoidTask";
    attr.attr_bits  = 0U;
    attr.cb_mem     = NULL;
    attr.cb_size    = 0U;
    attr.stack_mem  = NULL;
    attr.stack_size = TASK_STACK_SIZE;
    attr.priority   = TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)TapeAvoidTask, NULL, &attr) == NULL) {
        printf("[TapeAvoid] 创建任务失败!\r\n");
    }
}

APP_FEATURE_INIT(TapeAvoidEntry);
