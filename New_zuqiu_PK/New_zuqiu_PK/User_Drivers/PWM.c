#include "PWM.h"

void PWM_Init(void )
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
}


void PWM_SetCompare1(uint16_t Compare)//rightwheel
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Compare);
}


void PWM_SetCompare2(uint16_t Compare)//leftwheel
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Compare);
}


void PWM_SetCompare3(uint16_t Compare)//rightwheel
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Compare);
}


void PWM_SetCompare4(uint16_t Compare)//rightwheel
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Compare);
}


