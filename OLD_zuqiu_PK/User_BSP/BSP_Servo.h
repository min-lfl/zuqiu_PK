#ifndef __BSP_SERVO_H__
#define __BSP_SERVO_H__

//############头文件引用区###########
#include "PWM.h"


void Servo_Init(void);


void zuozhuan(void);//左转
void yuozhuan(void);//右转
void qianjin(void);//前进
void houtui(void);//后退
void tingzhi(void);//停止
void Full(void);//全速前进
void Step(void);//全速退后
void zuo(void);//全速左转
void you(void);//全速右转

#endif
