#include "motor.h"
#include "sys.h"
u8 AIN = 0;
u8 BIN = 0;
u16 PWMA = 0;
u16 PWMB = 0;
static u16 pwm_period = 0;
/*********************************************************
��������: ��ʼ���������
��ڲ���: ��
���� ֵ: ��
*********************************************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_13; //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);                 //�����趨������ʼ��GPIOB

	AIN=0;
	BIN=0;
}

/*********************************************************
��������: ��ʼ����������ʱ��PWM
��ڲ���: ��
���� ֵ: ��
*********************************************************/
void PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	pwm_period = arr + 1;
	Motor_Init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //ʹ��GPIOʱ��
	//���ø�����Ϊ�����������,���TIM4 CH1 CH2��PWM���岨��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //TIM4_CH1 //TIM4_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr; //�Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //Ԥ��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //��ʼ��TIM4

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                             //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //�������:��
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //��ʼ��TIM4 CH1
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //��ʼ��TIM4 CH2

	TIM_CtrlPWMOutputs(TIM4, ENABLE);	//MOE �����ʹ��

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //CH2Ԥװ��ʹ��

	TIM_ARRPreloadConfig(TIM4, ENABLE); //ʹ��TIM4��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
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
    //moto1->A���(����), moto2->B���(����)
    if(moto2>=0) {
        BIN=0;
        PWMB=myabs(moto2);
        GPIO_ResetBits(GPIOB,GPIO_Pin_14);  //BIN�͵�ƽ ��תǰ��
    }else{
        BIN=1;
        PWMB=pwm_period-myabs(moto2);
        if(PWMB>=pwm_period) PWMB=0;
        GPIO_SetBits(GPIOB,GPIO_Pin_14);    //BIN�ߵ�ƽ ��ת����
    }

    if(moto1>=0) {
        AIN=0;
        PWMA=myabs(moto1);
        GPIO_ResetBits(GPIOB,GPIO_Pin_13);  //AIN�͵�ƽ ��תǰ��
    }else{
        AIN=1;
        PWMA=pwm_period-myabs(moto1);
        if(PWMA>=pwm_period) PWMA=0;
        GPIO_SetBits(GPIOB,GPIO_Pin_13);    //AIN�ߵ�ƽ ��ת����
    }
    TIM_SetCompare1(TIM4, PWMA);
    TIM_SetCompare2(TIM4, PWMB);
}
