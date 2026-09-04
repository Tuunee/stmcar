/*
 * 16.0_TapeAvoid - two-sensor black-tape line follower
 *
 * GPIO13/14 are the left/right TCRT5000 comparator outputs. The current
 * board was measured as active-high on black tape. Set IR_BLACK_LEVEL to 0

 
 * when using a board with the opposite comparator polarity.
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

#define CAR_UART_IDX       HI_UART_IDX_2
#define IR_LEFT_GPIO       13
#define IR_RIGHT_GPIO      14
#define IR_BLACK_LEVEL     1

#define LINE_SPEED         68
#define SHARP_TURN_SPEED   72
#define LINE_SEARCH_SPEED  60
#define BOTH_BLACK_SPEED   58
#define MIN_MOTOR_SPEED    55
#define RIGHT_COMP_PCT     15

#define CONTROL_TICK       5       /* 50 ms on this board */
#define KEEPALIVE_TICKS    10
#define JUNCTION_DEBOUNCE  2       /* 100 ms of both sensors on black */
#define JUNCTION_REARM     4       /* 200 ms away from the junction */
#define SHARP_TURN_LIMIT   80      /* 4 seconds maximum search time */
#define TURN_MIN_TICKS     6       /* 300 ms, clear the junction first */
#define TASK_STACK_SIZE    (1024 * 4)
#define TASK_PRIO          25

static uint8_t txbuf[6];

enum LineError {
    LINE_LOST = 2,
    LINE_LEFT = -1,
    LINE_CENTER = 0,
    LINE_RIGHT = 1,
};

static void SendCar(int motor_left, int motor_right)
{
    uint8_t left_dir = 0;
    uint8_t right_dir = 0;

    if (motor_left < 0) {
        left_dir = 1;
        motor_left = -motor_left;
    }
    if (motor_right < 0) {
        right_dir = 1;
        motor_right = -motor_right;
    }

    motor_right = motor_right * (100 + RIGHT_COMP_PCT) / 100;
    if (motor_left > 0 && motor_left < MIN_MOTOR_SPEED) {
        motor_left = MIN_MOTOR_SPEED;
    }
    if (motor_right > 0 && motor_right < MIN_MOTOR_SPEED) {
        motor_right = MIN_MOTOR_SPEED;
    }
    if (motor_left > 150) {
        motor_left = 150;
    }
    if (motor_right > 150) {
        motor_right = 150;
    }

    txbuf[0] = 0xFC;
    txbuf[1] = left_dir;
    txbuf[2] = (uint8_t)motor_left;
    txbuf[3] = right_dir;
    txbuf[4] = (uint8_t)motor_right;
    txbuf[5] = 0xFD;
    UartWrite(CAR_UART_IDX, txbuf, 6);
}

static int IsBlack(WifiIotGpioValue value)
{
    return ((int)value == IR_BLACK_LEVEL);
}

static int ReadLineError(int *left_black, int *right_black)
{
    WifiIotGpioValue left_value;
    WifiIotGpioValue right_value;

    GpioGetInputVal(IR_LEFT_GPIO, &left_value);
    GpioGetInputVal(IR_RIGHT_GPIO, &right_value);
    *left_black = IsBlack(left_value);
    *right_black = IsBlack(right_value);

    if (*left_black && !*right_black) {
        return LINE_LEFT;
    }
    if (!*left_black && *right_black) {
        return LINE_RIGHT;
    }
    if (!*left_black && !*right_black) {
        return LINE_CENTER;
    }
    return LINE_LOST;
}

static void FollowLine(int error, int previous_error, int left_black, int right_black)
{
    int motor_left = LINE_SPEED;
    int motor_right = LINE_SPEED;

    if (left_black && right_black) {
        motor_left = BOTH_BLACK_SPEED;
        motor_right = BOTH_BLACK_SPEED;
    } else if (error == LINE_LEFT) {
        motor_left = -SHARP_TURN_SPEED / 2;
        motor_right = SHARP_TURN_SPEED;
    } else if (error == LINE_RIGHT) {
        motor_left = SHARP_TURN_SPEED;
        motor_right = -SHARP_TURN_SPEED / 2;
    } else if (previous_error < 0) {
        motor_left = -LINE_SEARCH_SPEED / 2;
        motor_right = LINE_SEARCH_SPEED;
    } else if (previous_error > 0) {
        motor_left = LINE_SEARCH_SPEED;
        motor_right = -LINE_SEARCH_SPEED / 2;
    }

    SendCar(motor_left, motor_right);
}

enum FollowMode {
    MODE_FOLLOW = 0,
    MODE_TURN_LEFT,
    MODE_TURN_RIGHT,
    MODE_FINISHED,
};

static void TapeAvoidTask(void)
{
    int previous_error = LINE_CENTER;
    int left_black;
    int right_black;
    int error;
    int tick = 0;
    int junction_count = 0;
    int junction_black_ticks = 0;
    int junction_white_ticks = JUNCTION_REARM;
    int junction_armed = 0;
    int start_white_ticks = 0;
    int turn_ticks = 0;
    enum FollowMode mode = MODE_FOLLOW;

    printf("[TapeAvoid] line following started, black level=%d\r\n",
           IR_BLACK_LEVEL);

    while (1) {
        osDelay(CONTROL_TICK);
        error = ReadLineError(&left_black, &right_black);

        if (mode == MODE_FINISHED) {
            SendCar(0, 0);
            continue;
        }

        /* Keep turning until the new branch is found; never fall through to
         * FollowLine with stale sensor values during this manoeuvre. */
        if (mode == MODE_TURN_LEFT || mode == MODE_TURN_RIGHT) {
            turn_ticks++;
            if (mode == MODE_TURN_LEFT) {
                SendCar(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
            } else {
                SendCar(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);
            }

            if (turn_ticks >= TURN_MIN_TICKS &&
                ((mode == MODE_TURN_LEFT && left_black && !right_black) ||
                 (mode == MODE_TURN_RIGHT && !left_black && right_black))) {
                mode = MODE_FOLLOW;
                turn_ticks = 0;
                junction_black_ticks = 0;
                junction_white_ticks = 0;
                junction_armed = 0;
                SendCar(LINE_SPEED, LINE_SPEED);
            } else if (turn_ticks >= SHARP_TURN_LIMIT) {
                /* A sensor miss must not leave the motors in an undefined
                 * state; resume forward motion and let line search recover. */
                printf("[TapeAvoid] turn timeout, resume line search\r\n");
                mode = MODE_FOLLOW;
                turn_ticks = 0;
                junction_black_ticks = 0;
                junction_white_ticks = 0;
                junction_armed = 0;
                SendCar(LINE_SPEED, LINE_SPEED);
            }
            continue;
        }

        /* The starting marker is also double black; leave it before arming. */
        if (!left_black || !right_black) {
            if (start_white_ticks < JUNCTION_REARM) {
                start_white_ticks++;
            }
            if (start_white_ticks >= JUNCTION_REARM) {
                junction_armed = 1;
            }
        }

        if (left_black && right_black) {
            junction_black_ticks++;
            junction_white_ticks = 0;
            if (junction_armed && junction_black_ticks >= JUNCTION_DEBOUNCE) {
                junction_armed = 0;
                junction_count++;
                if (junction_count == 1) {
                    printf("[TapeAvoid] junction 1: choose LEFT\r\n");
                    mode = MODE_TURN_LEFT;
                    turn_ticks = 0;
                } else if (junction_count == 2) {
                    printf("[TapeAvoid] junction 2: choose RIGHT\r\n");
                    mode = MODE_TURN_RIGHT;
                    turn_ticks = 0;
                } else {
                    printf("[TapeAvoid] finish marker reached, stop\r\n");
                    mode = MODE_FINISHED;
                    SendCar(0, 0);
                    continue;
                }
                continue;
            }
        } else {
            junction_black_ticks = 0;
            junction_white_ticks++;
            if (junction_white_ticks >= JUNCTION_REARM) {
                junction_armed = 1;
            }
        }

        if (error != LINE_CENTER && error != LINE_LOST) {
            previous_error = error;
        }
        FollowLine(error, previous_error, left_black, right_black);

        tick += CONTROL_TICK;
        if (tick >= KEEPALIVE_TICKS * CONTROL_TICK) {
            tick = 0;
            printf("[TapeAvoid] L=%d R=%d err=%d\r\n",
                   left_black, right_black, error);
        }
    }
}

static void TapeAvoidEntry(void)
{
    uint32_t ret;
    WifiIotUartAttribute car_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    osThreadAttr_t attr = {0};

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    ret = UartInit(CAR_UART_IDX, &car_attr, NULL);
    if (ret != WIFI_IOT_SUCCESS) {
        printf("[TapeAvoid] UART2 init failed: 0x%x\r\n", ret);
        return;
    }

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(IR_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(IR_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);

    attr.name = "TapeAvoidTask";
    attr.stack_size = TASK_STACK_SIZE;
    attr.priority = TASK_PRIO;
    if (osThreadNew((osThreadFunc_t)TapeAvoidTask, NULL, &attr) == NULL) {
        printf("[TapeAvoid] task creation failed\r\n");
    }
}

APP_FEATURE_INIT(TapeAvoidEntry);
