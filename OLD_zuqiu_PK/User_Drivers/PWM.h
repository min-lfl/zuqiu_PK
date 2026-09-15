#ifndef _PWM_H
#define _PWM_H

#include "main.h"
#include "tim.h"


void PWM_Init(void);

void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);

#endif
