#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * =============================================================================
 *                         硬件配置与标定参数
 * =============================================================================
 * 所有硬件绑定关系和底盘标定参数都集中放在这里。
 * 后续更换定时器、调整接线或者重新标定底盘时，通常只需要修改本区域。
 */

/* 四个车轮 PWM 输出共用的定时器句柄。 */
#define BSP_SERVO_TIMER_HANDLE             htim4

/*
 * PWM 通道与车轮的硬件接线关系：
 *   CH1 -> 右前轮（FR），CH2 -> 左前轮（FL）
 *   CH3 -> 左后轮（RL），CH4 -> 右后轮（RR）
 */
#define BSP_SERVO_FR_PWM_CHANNEL           TIM_CHANNEL_1
#define BSP_SERVO_FL_PWM_CHANNEL           TIM_CHANNEL_2
#define BSP_SERVO_RL_PWM_CHANNEL           TIM_CHANNEL_3
#define BSP_SERVO_RR_PWM_CHANNEL           TIM_CHANNEL_4

/*
 * 四个车轮的独立中位标定值，数值为绝对 CCR 值。
 *
 * 由于器件误差和老化，CCR=1500 不一定能让每个电机完全停止。
 * 应分别测量四个车轮真正的停车值，然后只修改对应车轮的宏。
 * 在没有实测数据之前，四个车轮均使用标准中位值 1500。
 */
#define BSP_SERVO_FR_NEUTRAL_CCR           (1500U)
#define BSP_SERVO_FL_NEUTRAL_CCR           (1500U)
#define BSP_SERVO_RL_NEUTRAL_CCR           (1500U)
#define BSP_SERVO_RR_NEUTRAL_CCR           (1500U)

/*
 * 相对于各车轮独立中位值的最大输出幅度。
 * 数值 250 会把输出限制在 [中位值 - 250，中位值 + 250] 内，
 * 既符合驱动板标称的 1250～1750 输入范围，也允许每个车轮使用不同中位值。
 */
#define BSP_SERVO_MAX_OUTPUT_AMP           (250U)

/*
 * 机械死区补偿，单位为 CCR 计数值。
 * 任意非零速度指令都会先跨过这 40 个计数的死区，使较小指令也能启动电机。
 * 输入为零时不添加死区补偿，始终输出该车轮精确标定后的中位值。
 */
#define BSP_SERVO_DEAD_ZONE                (40U)

/* 速度指令满量程：10000 表示 100.00%。 */
#define BSP_SERVO_SPEED_FULL_SCALE         (10000)

/*
 * 底盘机械尺寸，单位为厘米，应测量左右/前后车轮接地点中心之间的距离。
 * 混控函数按照下面的公式计算内侧轮补偿系数：
 *
 *                    k = WHEEL_BASE / TRACK_WIDTH
 *
 * 当前两个默认尺寸相等，因此 k=1.0；在获得实测尺寸之前，相当于不进行
 * 额外补偿，只使用标准滑移转向混控。填入实际正数尺寸后即可启用机械补偿。
 * 在当前简化模型中，0 < k <= 1 时可以按预期降低内侧轮速度。
 */
#define BSP_CHASSIS_TRACK_WIDTH_CM         (20.0F)
#define BSP_CHASSIS_WHEEL_BASE_CM          (20.0F)

/* 以下参数关系如果不成立，死区映射公式将失去意义，因此在编译期直接报错。 */
#if (BSP_SERVO_DEAD_ZONE > BSP_SERVO_MAX_OUTPUT_AMP)
#error "BSP_SERVO_DEAD_ZONE must not exceed BSP_SERVO_MAX_OUTPUT_AMP"
#endif

#if ((BSP_SERVO_FR_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP) || \
     (BSP_SERVO_FL_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP) || \
     (BSP_SERVO_RL_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP) || \
     (BSP_SERVO_RR_NEUTRAL_CCR < BSP_SERVO_MAX_OUTPUT_AMP))
#error "Each neutral CCR must be at least BSP_SERVO_MAX_OUTPUT_AMP"
#endif

/*
 * 底层 PWM 写入封装。应用层应调用本函数，不应直接调用
 * __HAL_TIM_SET_COMPARE。非法通道不会执行写入，compare 也会被限制在
 * 当前定时器 ARR 范围内。
 */
void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare);

/*
 * 四个车轮的独立控制接口。
 * speed_percent 使用万分比，允许范围为 [-10000, 10000]：
 *   +10000 = 全速前进，0 = 标定中位停车，-10000 = 全速后退。
 */
void BSP_Servo_SetFrontRightWheelSpeed(int16_t speed_percent);
void BSP_Servo_SetFrontLeftWheelSpeed(int16_t speed_percent);
void BSP_Servo_SetRearLeftWheelSpeed(int16_t speed_percent);
void BSP_Servo_SetRearRightWheelSpeed(int16_t speed_percent);

/*
 * 四轮滑移转向混控接口。
 * throttle：正数前进、负数后退；steering：正数右转、负数左转。
 * 两个输入的有效范围都是 [-10000, 10000]。
 */
void BSP_Chassis_Drive(int16_t throttle, int16_t steering);

/* 使用四轮各自的停车中位值启动全部 PWM 通道，确保上电时底盘静止。 */
void BSP_Servo_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
