#include "BSP_SERVO.H"


void Servo_Init(void){
	PWM_Init();
}

void zuozhuan(void)//左转
{
	PWM_SetCompare1(1600);//左前
	PWM_SetCompare2(1600);//右前
  PWM_SetCompare3(1600);//左后
  PWM_SetCompare4(1600);//右后
}
void yuozhuan(void)//右转
{
	PWM_SetCompare1(1400);//左前
	PWM_SetCompare2(1400);//右前
  PWM_SetCompare3(1400);//左后
  PWM_SetCompare4(1400);//右后
}
void qianjin(void)//前进
{
	PWM_SetCompare1(1350);//左前
	PWM_SetCompare2(1650);//右前
  PWM_SetCompare3(1350);//左后
  PWM_SetCompare4(1650);//右后
}
void houtui(void)//后退
{
	PWM_SetCompare1(1650);//左前
	PWM_SetCompare2(1350);//右前
  PWM_SetCompare3(1650);//左后
  PWM_SetCompare4(1350);//右后
}
void tingzhi(void)//停止
{
	PWM_SetCompare1(1500);//左前
	PWM_SetCompare2(1500);//右前
  PWM_SetCompare3(1500);//左后
  PWM_SetCompare4(1500);//右后
}
void Full(void)//全速前进
{
	PWM_SetCompare1(1270);//左前
	PWM_SetCompare2(1720);//右前
  PWM_SetCompare3(1270);//左后
  PWM_SetCompare4(1720);//右后
}
void Step(void)//全速退后
{
	PWM_SetCompare1(1720);//左前
	PWM_SetCompare2(1270);//右前
  PWM_SetCompare3(1720);//左后
  PWM_SetCompare4(1270);//右后
}
void zuo(void)//全速左转
{
	PWM_SetCompare1(1720);//左前
	PWM_SetCompare2(1720);//右前
  PWM_SetCompare3(1720);//左后
  PWM_SetCompare4(1720);//右后
}
void you(void)//全速右转
{
	PWM_SetCompare1(1270);//左前
	PWM_SetCompare2(1270);//右前
  PWM_SetCompare3(1270);//左后
  PWM_SetCompare4(1270);//右后
}

