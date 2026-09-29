#include "stm32f10x.h"                  // Device header
#include "Servo.h"
#include "Delay.h"
#include "usart1.h"
#include "PWM.h"
#include "beep.h"
#include "bsp_advance_tim.h"
#include "bsp_TiMbase.h" 

uint16_t tim_mnu;
u8 djdj,dnum;
extern u8 beep;
extern u8 mun_1;

	
int main(void)
{
	E49_GPIO_Config();//配置E49传输模式
	uart_init(9600);
	BEEP_Config();
	PWM_Init();
	tingzhi();
	ADVANCE_TIM1_Init();
	TIM4_PWM_Init(19999,71);
	BASIC_TIM_Init();
//	Pwm_Tim4_CH3(60);//大臂200最高55最低
//	Pwm_Tim4_CH2(200);//夹子  270最大200最小
//	Pwm_Tim4_CH4(220);//小臂220最低
	

	while(1)
	{
		PWM_SetCompare1(500);
		if(djdj==1)//下降
		{ 
			Pwm_Tim4_CH4(220);	//小臂 
			Pwm_Tim4_CH3(60);  //大臂
			Delay_ms(150);     
			Pwm_Tim4_CH2(270); //夹子
			
			djdj==0;
			
			
		}
				if(djdj==2)//上升
		{ 
			Pwm_Tim4_CH2(200);  //夹子
			Delay_ms(350);//原500
			Pwm_Tim4_CH4(260);  //小臂
			Pwm_Tim4_CH3(200);  //大臂
			
			djdj==0;
		}
				if(djdj==3)//停止
		{
	  PWM_SetCompare1(1450);//左前
	  PWM_SetCompare2(1450);//右前
    PWM_SetCompare3(1450);//左后
    PWM_SetCompare4(1450);//右后
			
		djdj==0;
		}

  }
}

void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET )
	{
		if((mun_1>0)&&(mun_1<4))
		{
			tim_mnu++;
			if(tim_mnu>=200)
			{
				tingzhi();//停止
				tim_mnu=0;
				mun_1=0;
			}
		}
		
	  TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);  		 
	}
}
