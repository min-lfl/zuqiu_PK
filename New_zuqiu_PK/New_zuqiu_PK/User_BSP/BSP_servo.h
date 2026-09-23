#ifndef BSP_servo_H
#define BSP_servo_H

#ifdef __cplusplus
extern "C" {
#endif

//###############头文件引用区####################
#include <stdint.h>
#include "tim.h"

//###############宏定义区域######################
// 舵机关节定时器宏定义
#define BSP_SERVO_TIMER_HANDLE			htim2


/*
* 机械爪共有 3 个关节, 1 个夹爪，外形大致如下(侧视图,关节只能上下转动)：
 *
 *                                          6.2cm            7.3cm            9.00cm
 *               小车             (关节1)----------(关节2)-----------(关节3)-----------<爪子>
 * ------------------------------------                  
 * |                                  |  
 * |                                  |                地面到关节1中心点的高度为9.58cm
 * |                                  |
 * ----- 轮子 -------------轮子--------
 *
 * 这是关节伸长到最长时的侧面图像,关节只能在小车前进轴上运动
 * 舵机的控制和sg90控制一样,20ms周期里,单周期内不同的高电平对应不同角度
 * 0.5MS---------0度;
 * 1.0MS--------45度;
 * 1.5MS--------90度;
 * 2.0MS-------135度;
 * 2.5MS-------180度;
 */
// 机械爪舵机关节宏定义
// 机械臂关节舵机 (J = Joint)
#define BSP_Servo_J1_PWM_CHANNEL           TIM_CHANNEL_1  // 关节1/底座
#define BSP_Servo_J2_PWM_CHANNEL           TIM_CHANNEL_2  // 关节2
#define BSP_Servo_J3_PWM_CHANNEL           TIM_CHANNEL_3  // 关节3
// 夹爪舵机
#define BSP_Servo_Gripper_PWM_CHANNEL      TIM_CHANNEL_4  // 夹子 (也可以用 BSP_Servo_Claw_PWM_CHANNEL)


//#################调试接口函数区###################

void BSP_Servo_SetJ1Pulse(uint16_t pulse_val);	//写入通道1的ccr值,控制关节1
void BSP_Servo_SetJ2Pulse(uint16_t pulse_val);	//写入通道2的ccr值,控制关节2
void BSP_Servo_SetJ3Pulse(uint16_t pulse_val);	//写入通道3的ccr值,控制关节3
void BSP_Servo_SetGripperPulse(uint16_t speed_percent);	//写入通道4的ccr值,控制关夹爪

//##################用户函数区域####################
void BSP_Servo_Init(void);	





#ifdef __cplusplus
}
#endif

#endif
