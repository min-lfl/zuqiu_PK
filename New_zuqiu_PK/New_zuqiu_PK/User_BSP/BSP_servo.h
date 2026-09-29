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


//###############舵机控制与标定参数###############
//TIM2 每个计数对应约 1us，因此标准舵机脉宽 0.5ms～2.5ms 对应 CCR 500～2500。
//绝对角控制函数生成的关节 CCR 会限制在这个范围内，防止算法输出超出舵机范围。
//直接调用下面的 SetJxPulse/SetGripperPulse 调试接口时仍可写入 0～ARR 的原始 CCR。
#define BSP_SERVO_MIN_PULSE_CCR             (500U)
#define BSP_SERVO_MAX_PULSE_CCR             (2500U)

//舵机从 0 度转到 180 度时，CCR 从 500 变化到 2500，共变化 2000。
//使用浮点数保存每度对应的 CCR，既容易看懂，也可避免整数除法损失精度。
#define BSP_SERVO_ANGLE_RANGE_DEG           (180.0F)
#define BSP_SERVO_PULSE_RANGE_CCR           (2000.0F)
#define BSP_SERVO_CCR_PER_DEGREE            \
    (BSP_SERVO_PULSE_RANGE_CCR / BSP_SERVO_ANGLE_RANGE_DEG)

/*
 * 三个关节的水平零点标定值，单位为 CCR。
 * 调整方法：分别手动写入三个关节的 CCR，使 AB、BC、CD 全部共线且平行于地面，
 * 然后把测得的三个 CCR 填到这里。例如测得 1340、1230、1840，就分别替换下列值。
 */
#define BSP_SERVO_J1_HORIZONTAL_CCR         (1500U)
#define BSP_SERVO_J2_HORIZONTAL_CCR         (1500U)
#define BSP_SERVO_J3_HORIZONTAL_CCR         (1500U)

/*
 * 三个关节的安装方向，实机测试后每项只能填写 +1 或 -1。
 * +1：局部关节角增大时 CCR 应增大；-1：局部关节角增大时 CCR 应减小。
 * 这里的“局部关节角”是相邻两段连杆之间的角度，不是三段连杆的地面绝对角。
 */
#define BSP_SERVO_J1_DIRECTION              (+1)
#define BSP_SERVO_J2_DIRECTION              (+1)
#define BSP_SERVO_J3_DIRECTION              (+1)

//配置参数检查，避免方向或标定值填写错误后仍然带着错误参数运行。
#if (((BSP_SERVO_J1_DIRECTION != 1) && (BSP_SERVO_J1_DIRECTION != -1)) || \
     ((BSP_SERVO_J2_DIRECTION != 1) && (BSP_SERVO_J2_DIRECTION != -1)) || \
     ((BSP_SERVO_J3_DIRECTION != 1) && (BSP_SERVO_J3_DIRECTION != -1)))
#error "Each servo direction must be +1 or -1"
#endif

#if ((BSP_SERVO_MIN_PULSE_CCR >= BSP_SERVO_MAX_PULSE_CCR) || \
     (BSP_SERVO_J1_HORIZONTAL_CCR < BSP_SERVO_MIN_PULSE_CCR) || \
     (BSP_SERVO_J1_HORIZONTAL_CCR > BSP_SERVO_MAX_PULSE_CCR) || \
     (BSP_SERVO_J2_HORIZONTAL_CCR < BSP_SERVO_MIN_PULSE_CCR) || \
     (BSP_SERVO_J2_HORIZONTAL_CCR > BSP_SERVO_MAX_PULSE_CCR) || \
     (BSP_SERVO_J3_HORIZONTAL_CCR < BSP_SERVO_MIN_PULSE_CCR) || \
     (BSP_SERVO_J3_HORIZONTAL_CCR > BSP_SERVO_MAX_PULSE_CCR))
#error "Servo horizontal CCR must be within the configured pulse range"
#endif


//#################调试接口函数区###################

void BSP_Servo_SetJ1Pulse(uint16_t pulse_val);	//写入通道1的ccr值,控制关节1
void BSP_Servo_SetJ2Pulse(uint16_t pulse_val);	//写入通道2的ccr值,控制关节2
void BSP_Servo_SetJ3Pulse(uint16_t pulse_val);	//写入通道3的ccr值,控制关节3
void BSP_Servo_SetGripperPulse(uint16_t pulse_val);	//写入通道4的ccr值,控制夹爪

//##################用户函数区域####################
void BSP_Servo_Init(void);

/**
 * @brief 设置 AB、BC、CD 三段连杆相对于地面的绝对角度。
 * @param ab_angle_deg AB 相对地面 +X 轴的角度，向上为正、向下为负。
 * @param bc_angle_deg BC 相对地面 +X 轴的角度，向上为正、向下为负。
 * @param cd_angle_deg CD 相对地面 +X 轴的角度，向上为正、向下为负。
 * @note  三个参数都传 0.0F 时，机械臂运动到三个水平标定 CCR 对应的共线姿态。
 *        超过舵机脉宽能力的结果会被限制在 BSP_SERVO_MIN/MAX_PULSE_CCR 内。
 */
void BSP_Servo_SetAbsoluteAngles(float ab_angle_deg,
                                 float bc_angle_deg,
                                 float cd_angle_deg);





#ifdef __cplusplus
}
#endif

#endif
