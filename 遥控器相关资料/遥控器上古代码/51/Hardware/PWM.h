#ifndef _PWM_H
#define _PWM_H
#include "stm32f10x.h"

void PWM_Init(void );
void TIM4_PWM_Init(u16 arr,u16 psc);

void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);

void zuozhuan(void);//左转
void yuozhuan(void);//右转
void qianjin(void);//前进
void houtui(void);//后退
void tingzhi(void);//停止

void Pwm_Tim4_CH1(uint16_t compioni2);
void Pwm_Tim4_CH2(uint16_t compioni2);
void Pwm_Tim4_CH3(uint16_t compioni2);
void Pwm_Tim4_CH4(uint16_t compioni2);
#endif
