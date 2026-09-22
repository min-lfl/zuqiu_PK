#ifndef BSP_Motor_H
#define BSP_Motor_H

#ifdef __cplusplus
extern "C" {
#endif

//###############头文件引用区#################
#include <stdint.h>
#include "tim.h"

//###############宏定义区域######################
#define BSP_SERVO_TIMER_HANDLE             htim4


//###############硬件配置与标定参数##############
//PWM 通道与车轮的硬件接线关系：
//CH1 -> 右前轮（FR）
//CH2 -> 左前轮（FL）
//CH3 -> 左后轮（RL）
//CH4 -> 右后轮（RR）
#define BSP_SERVO_FR_PWM_CHANNEL           TIM_CHANNEL_1
#define BSP_SERVO_FL_PWM_CHANNEL           TIM_CHANNEL_2
#define BSP_SERVO_RL_PWM_CHANNEL           TIM_CHANNEL_3
#define BSP_SERVO_RR_PWM_CHANNEL           TIM_CHANNEL_4

//四轮“逻辑正速度”映射到 CCR 增减方向的极性
//+1：正速度使 CCR 增大；-1：正速度使 CCR 减小
//根据整车实测：右侧物理前进需要 CCR 增大，左侧物理前进需要 CCR 减小
//经过本层校正后，四个单轮接口中的正速度都统一表示“小车前进方向”
#define BSP_SERVO_FR_FORWARD_POLARITY      (+1)
#define BSP_SERVO_FL_FORWARD_POLARITY      (-1)
#define BSP_SERVO_RL_FORWARD_POLARITY      (-1)
#define BSP_SERVO_RR_FORWARD_POLARITY      (+1)

//四轮的独立中位标定值，单位为 CCR 计数值。
//由于器件误差和老化，CCR=1500 不一定能让每个电机完全停止。
//分别测量四个车轮真正的停车值，比如1440时才停车就填1440
#define BSP_SERVO_FR_NEUTRAL_CCR           (1440U)
#define BSP_SERVO_FL_NEUTRAL_CCR           (1440U)
#define BSP_SERVO_RL_NEUTRAL_CCR           (1440U)
#define BSP_SERVO_RR_NEUTRAL_CCR           (1440U)

//相对于各车轮独立中位值的最大输出幅度，单位为 CCR 计数值
//数值 250 会把输出限制在 [中位值 - 250，中位值 + 250] 内，
//既符合驱动板标称的 1250～1750 输入范围，也允许每个车轮使用不同中位值。
#define BSP_SERVO_MAX_OUTPUT_AMP           (250U)

//机械死区补偿，单位为 CCR 计数值。
//这里填刚好可以让轮子动起来的CCR的值,我这边填20就能动
#define BSP_SERVO_DEAD_ZONE                (20U)

//速度指令满量程：10000 表示0-10000的可调速度,这个宏定义也用于限幅
#define BSP_SERVO_SPEED_FULL_SCALE         (10000)

//底盘机械尺寸，单位为厘米
//分别是左右轮距,前后轴距
//按尺子读数填写
//k=轴距/轮距只作为内侧轮衰减系数使用；若计算结果大于1，代码会限制为1
#define BSP_CHASSIS_TRACK_WIDTH_CM         (15.5F)
#define BSP_CHASSIS_WHEEL_BASE_CM          (11.5F)

//###############加减速斜坡参数####################
//斜坡算法的最小更新周期,单位ms；函数可以高频调用,但只会按此周期更新输出
#define BSP_CHASSIS_RAMP_UPDATE_PERIOD_MS        (10U)
//主循环偶然阻塞后允许参与计算的最大时间,防止dt过大导致速度突然跳变
#define BSP_CHASSIS_RAMP_MAX_DT_MS               (20U)
//前后油门加减速度,单位为“速度指令/秒”；20000表示约0.5秒从0加速到10000
#define BSP_CHASSIS_RAMP_THROTTLE_ACCEL_PER_SEC          (20000U)

//底盘能够开始转动的最小转向指令；低于该值时只有电机声音而车身基本不动
#define BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND            (1900)
//斜坡接口允许输出的最大转向指令，避免长按时原地高速旋转失控
#define BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND            (4500)
//组合行驶时，油门达到该值后才允许直接进入最小有效转向
//该值不能小于最小转向指令，否则转向可能大于油门并让内侧轮反转
#define BSP_CHASSIS_RAMP_COMBINED_MIN_THROTTLE           \
    (BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND)
//刚开始转向时的精细加速度；点按时从最小有效值开始缓慢增加
#define BSP_CHASSIS_RAMP_STEERING_ACCEL_PER_SEC          (1500U)
//长按过程中逐渐接近的最大加速度；代码会在精细值与该值之间连续插值
#define BSP_CHASSIS_RAMP_STEERING_FAST_ACCEL_PER_SEC     (5000U)
//松开、减小转向或反向操作时的减速度；低于最小有效值后会直接回到0
#define BSP_CHASSIS_RAMP_STEERING_BRAKE_PER_SEC          (80000U)

//参数检查逻辑,死区不能大于限幅
#if (BSP_SERVO_DEAD_ZONE > BSP_SERVO_MAX_OUTPUT_AMP)
#error "BSP_SERVO_DEAD_ZONE must not exceed BSP_SERVO_MAX_OUTPUT_AMP"
#endif
#if ((BSP_SERVO_FR_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP) || \
     (BSP_SERVO_FL_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP) || \
     (BSP_SERVO_RL_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP) || \
     (BSP_SERVO_RR_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP))
#error "Each neutral CCR must be at least BSP_SERVO_MAX_OUTPUT_AMP"
#endif
#if (((BSP_SERVO_FR_FORWARD_POLARITY != 1) && \
      (BSP_SERVO_FR_FORWARD_POLARITY != -1)) || \
     ((BSP_SERVO_FL_FORWARD_POLARITY != 1) && \
      (BSP_SERVO_FL_FORWARD_POLARITY != -1)) || \
     ((BSP_SERVO_RL_FORWARD_POLARITY != 1) && \
      (BSP_SERVO_RL_FORWARD_POLARITY != -1)) || \
     ((BSP_SERVO_RR_FORWARD_POLARITY != 1) && \
      (BSP_SERVO_RR_FORWARD_POLARITY != -1)))
#error "Each wheel forward polarity must be +1 or -1"
#endif
#if (BSP_CHASSIS_RAMP_UPDATE_PERIOD_MS == 0U)
#error "BSP_CHASSIS_RAMP_UPDATE_PERIOD_MS must be greater than zero"
#endif
#if (BSP_CHASSIS_RAMP_MAX_DT_MS < BSP_CHASSIS_RAMP_UPDATE_PERIOD_MS)
#error "BSP_CHASSIS_RAMP_MAX_DT_MS must not be less than update period"
#endif
#if ((BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND <= 0) || \
     (BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND >= BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND))
#error "Steering minimum command must be greater than zero and less than maximum"
#endif
#if (BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND > BSP_SERVO_SPEED_FULL_SCALE)
#error "Steering maximum command exceeds servo full scale"
#endif
#if ((BSP_CHASSIS_RAMP_COMBINED_MIN_THROTTLE < BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND) || \
     (BSP_CHASSIS_RAMP_COMBINED_MIN_THROTTLE > BSP_SERVO_SPEED_FULL_SCALE))
#error "Combined turn throttle threshold must be between steering minimum and full scale"
#endif
#if (BSP_CHASSIS_RAMP_STEERING_FAST_ACCEL_PER_SEC < BSP_CHASSIS_RAMP_STEERING_ACCEL_PER_SEC)
#error "Steering fast acceleration must not be less than fine acceleration"
#endif
#if (BSP_CHASSIS_RAMP_STEERING_BRAKE_PER_SEC == 0U)
#error "Steering brake rate must be greater than zero"
#endif


//#################调试接口函数区###################
//四个车轮的独立控制接口,有效范围都是 [-10000, 10000]
//经过物理方向极性校正后：正数统一表示小车前进方向，负数统一表示后退方向
void BSP_Servo_SetFrontRightWheelSpeed(int16_t speed_percent);		//右边前面轮子
void BSP_Servo_SetFrontLeftWheelSpeed(int16_t speed_percent);			//左边前面轮子
void BSP_Servo_SetRearLeftWheelSpeed(int16_t speed_percent);			//左边后面的轮子
void BSP_Servo_SetRearRightWheelSpeed(int16_t speed_percent);			//右边后面的轮子

//##################用户函数区域#################
void BSP_Servo_Init(void);																				//初始化全部 PWM 通道,并且确保上电时底盘静止。
void BSP_Chassis_Drive(int16_t throttle, int16_t steering);				//四轮滑移转向混控接口,参数分别是油门大小和左右转大小.
																																		//两个输入的有效范围都是 [-10000, 10000]。正负表方向
void BSP_Chassis_RampDrive(int16_t throttle, int16_t steering,
                           uint16_t throttle_acceleration,
                           uint16_t steering_acceleration); //油门和转向使用独立加速度
#ifdef __cplusplus
}
#endif

#endif
