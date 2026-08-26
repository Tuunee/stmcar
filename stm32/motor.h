#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "sys.h"

//方向控制全局变量，对应PB13 PB14
extern u8 AIN;
extern u8 BIN;

extern u16 PWMA;
extern u16 PWMB;

void Motor_Init(void);
void PWM_Init(u16 arr,u16 psc);
u32 myabs(long int a);
void Set_Pwm(int moto1,int moto2);


#endif
