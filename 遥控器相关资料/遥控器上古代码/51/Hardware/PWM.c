#include "stm32f10x.h"   // Device header
#include "PWM.h"
#include "Delay.h"


void PWM_Init(void )
{
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM2 ,ENABLE );//
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA ,ENABLE );//
 
	TIM_InternalClockConfig (TIM2 );//选择内部时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
	GPIO_Init(GPIOA,&GPIO_InitStructure );//
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure .TIM_ClockDivision = TIM_CKD_DIV1 ;
	TIM_TimeBaseInitStructure .TIM_CounterMode = TIM_CounterMode_Up ;
	TIM_TimeBaseInitStructure .TIM_Period = 20000 - 1;//ARR
	TIM_TimeBaseInitStructure .TIM_Prescaler = 72 - 1;//PSC
	TIM_TimeBaseInitStructure .TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit (TIM2 ,&TIM_TimeBaseInitStructure);//有一个更新事件，使预分频的值立刻有效
	//导致更新中断也因此发生 ，现象是一上电就进入中断
	TIM_ClearFlag (TIM2 ,TIM_FLAG_Update);//避免刚初始化就更新中断
	TIM_ITConfig (TIM2 ,TIM_IT_Update,ENABLE );
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCStructInit(&TIM_OCInitStructure);//防止成员未全部配初始值，而导致程序错误
	
	TIM_OCInitStructure .TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure .TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure .TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
	TIM_OCInitStructure .TIM_Pulse = 0;//CCR
	TIM_OC1Init(TIM2 ,&TIM_OCInitStructure);
	TIM_OC2Init(TIM2 ,&TIM_OCInitStructure);
	TIM_OC3Init(TIM2 ,&TIM_OCInitStructure);
	TIM_OC4Init(TIM2 ,&TIM_OCInitStructure);
	 
	TIM_Cmd(TIM2 ,ENABLE );
}


void PWM_SetCompare1(uint16_t Compare)//rightwheel
{
    TIM_SetCompare1 (TIM2,Compare);
}
void PWM_SetCompare2(uint16_t Compare)//leftwheel
{
    TIM_SetCompare2(TIM2,Compare);
}
void PWM_SetCompare3(uint16_t Compare)//rightwheel
{
    TIM_SetCompare3(TIM2,Compare);
}
void PWM_SetCompare4(uint16_t Compare)//rightwheel
{
    TIM_SetCompare4(TIM2,Compare);
}

void zuozhuan(void)//左转
{	
	PWM_SetCompare1(1600);//左前
	PWM_SetCompare2(1500);//右前
  PWM_SetCompare3(1600);//左后
  PWM_SetCompare4(1500);//右后
}
void yuozhuan(void)//右转
{
	PWM_SetCompare1(1400);//左前
	PWM_SetCompare2(1325);//右前
  PWM_SetCompare3(1400);//左后
  PWM_SetCompare4(1325);//右后

}
void qianjin(void)//前进
{
	PWM_SetCompare1(1580);//左前
	PWM_SetCompare2(1335);//右前
  PWM_SetCompare3(1580);//左后
  PWM_SetCompare4(1335);//右后
}
void houtui(void)//后退
{

	PWM_SetCompare1(1320);//左前
	PWM_SetCompare2(1580);//右前
  PWM_SetCompare3(1320);//左后
  PWM_SetCompare4(1580);//右后
}
void tingzhi(void)//停止
{
	PWM_SetCompare1(1450);//左前
	PWM_SetCompare2(1450);//右前
  PWM_SetCompare3(1450);//左后
  PWM_SetCompare4(1450);//右后
}

void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能定时器4时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能GPIO外设和AFIO复用功能模块时钟
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
    //初始化TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM4 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure .TIM_Pulse = 0;//CCR
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 

	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}
//void Pwm_Tim4_CH1(uint16_t compioni2)
//{
//	 compioni2=(float)compioni2*7.4074;
//	 compioni2+=500;
//	 if(compioni2>2500){compioni2=2500;}
//	 TIM_SetCompare1(TIM4,compioni2);
//}

void Pwm_Tim4_CH1(uint16_t compioni2)
{
	 compioni2=(float)compioni2*7.4074;
	 compioni2+=500;
	 if(compioni2>2500){compioni2=2500;}
	 TIM_SetCompare1(TIM4,compioni2);
}

void Pwm_Tim4_CH2(uint16_t compioni2)
{
	 compioni2=(float)compioni2*7.4074;
	 compioni2+=500;
	 if(compioni2>2500){compioni2=2500;}
	 TIM_SetCompare2(TIM4,compioni2);
}
void Pwm_Tim4_CH3(uint16_t compioni2)
{
	 compioni2=(float)compioni2*7.4074;
	 compioni2+=500;
	 if(compioni2>2500){compioni2=2500;}
	 TIM_SetCompare3(TIM4,compioni2);
}
void Pwm_Tim4_CH4(uint16_t compioni2)
{
	 compioni2=(float)compioni2*7.4074;
	 compioni2+=500;
	 if(compioni2>2500){compioni2=2500;}
	 TIM_SetCompare4(TIM4,compioni2);
}
