#include "beep.h"
#include "stm32f10x.h"

void BEEP_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA ,ENABLE );
	GPIO_InitStructure .GPIO_Mode =GPIO_Mode_Out_PP ;
	GPIO_InitStructure .GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure .GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init (GPIOA ,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA ,GPIO_Pin_8);
	
			
	// 开启定时器时钟,即内部时钟CK_INT=72M
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

/*--------------------时基结构体初始化-------------------------*/
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period= 20000-1;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= 72-1;	
	// 时钟分频因子 ，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// 输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 
	TIM_OCInitStructure.TIM_Pulse = 0;// 设置占空比大小
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	// 输出通道电平极性配置
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;// 输出通道空闲电平极性配置
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	// 使能计数器
	TIM_Cmd(TIM1, ENABLE);	
	// 主输出使能，当使用的是通用定时器时，这句不需要
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_DJ1(uint16_t Compare)
{
	Compare=(float)Compare*7.4074;
	Compare+=500;
	if(Compare>2500){Compare=2500;}
	TIM_SetCompare1 (TIM1,Compare);
}
void PWM_DJ2(uint16_t Compare)
{
	Compare=(float)Compare*7.4074;
	Compare+=500;
	if(Compare>2500){Compare=2500;}
	TIM_SetCompare4 (TIM1,Compare);
}

