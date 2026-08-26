#include "motor.h"
#include "sys.h"
u8 AIN = 0;
u8 BIN = 0;
u16 PWMA = 0;
u16 PWMB = 0;
/*********************************************************
函数功能: 初始化电机方向
入口参数: 无
返回 值: 无
*********************************************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_13; //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);                 //根据设定参数初始化GPIOB
	
	AIN=0;
	BIN=0;
}


/*********************************************************
函数功能: 初始化初始化定时器pwm
入口参数: 无
返回 值: 无
*********************************************************/
void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	Motor_Init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //使能GPIO外设时钟使能
	//设置该引脚为复用输出功能,输出TIM1 CH1 CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //TIM_CH1 //TIM_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                             //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_CtrlPWMOutputs(TIM4, ENABLE);	//MOE 主输出使能

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH1预装载使能
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH4预装载使能

	TIM_ARRPreloadConfig(TIM4, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM1
}
u32 myabs(long int a)
{
    u32 temp;
    if(a<0)
        temp=-a;
    else
        temp=a;

    return temp;
}

void Set_Pwm(int moto1,int moto2)
{
    //XIN PWMX在motro.h中有定义
    if(moto2>=0) {
        BIN=0;
        PWMB=myabs(moto2);
			GPIO_ResetBits(GPIOB,GPIO_Pin_14);  //BIN输出低电平，正转前进
    }else{
        BIN=1;
        PWMB=myabs(moto2);
			GPIO_SetBits(GPIOB,GPIO_Pin_14);    //BIN输出高电平，反转后退
    }

    if(moto1>=0) {
        AIN=0;
        PWMA=myabs(moto1);
			GPIO_ResetBits(GPIOB,GPIO_Pin_13);  //AIN输出低电平，正转前进
    }else{
        AIN=1;
        PWMA=myabs(moto1);
			GPIO_SetBits(GPIOB,GPIO_Pin_13);    //AIN输出高电平，反转后退
    }
		TIM_SetCompare1(TIM4, PWMA);
    TIM_SetCompare2(TIM4, PWMB);
}

