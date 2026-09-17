#include "BSP_Servo.h"

/* 在速度指令参与后续计算前，先将其限制在允许范围内。 */
static int32_t BSP_Servo_ClampCommand(int32_t command)
{
    if (command > BSP_SERVO_SPEED_FULL_SCALE)
    {
        return BSP_SERVO_SPEED_FULL_SCALE;
    }

    if (command < -BSP_SERVO_SPEED_FULL_SCALE)
    {
        return -BSP_SERVO_SPEED_FULL_SCALE;
    }

    return command;
}

/* 计算浮点数绝对值，避免为了一个简单操作而引入数学库。 */
static float BSP_Servo_AbsFloat(float value)
{
    return (value < 0.0F) ? -value : value;
}

/*
 * 将混控后的浮点指令转换回对外使用的整数格式。
 * 强制类型转换前，根据正负方向加上或减去 0.5，可以实现四舍五入，
 * 避免直接转换时总是向零截断。最后再次限幅，用于消除 +/-10000 附近
 * 可能出现的浮点舍入误差。
 */
static int16_t BSP_Servo_FloatToCommand(float command)
{
    int32_t rounded_command;

    if (command >= 0.0F)
    {
        rounded_command = (int32_t)(command + 0.5F);
    }
    else
    {
        rounded_command = (int32_t)(command - 0.5F);
    }

    return (int16_t)BSP_Servo_ClampCommand(rounded_command);
}

/*
 * 将单个车轮的速度指令转换为定时器使用的绝对 CCR 值。
 *
 * 对于非零输入 S，定义：
 *
 *   A = BSP_SERVO_MAX_OUTPUT_AMP
 *   D = BSP_SERVO_DEAD_ZONE
 *   F = BSP_SERVO_SPEED_FULL_SCALE = 10000
 *
 * 映射公式为：
 *
 *   effective_range = A - D
 *   linear_part     = effective_range * |S| / F
 *   compensated     = D + linear_part
 *   CCR             = neutral + sign(S) * compensated
 *
 * 因此，第一个非零指令就能跨过机械死区；当输入达到满量程时，输出仍然
 * 精确落在 neutral +/- A。公式计算完成后再次限幅，作为应对非法输入和
 * 浮点舍入误差的第二层安全保护。
 */
static uint16_t BSP_Servo_SpeedToCompare(int16_t speed_percent,
                                         uint16_t neutral_compare)
{
    int32_t command;
    int32_t magnitude;
    int32_t direction;
    int32_t compensated_offset;
    int32_t compare;
    int32_t minimum_compare;
    int32_t maximum_compare;
    float effective_range;
    float linear_part;

    command = BSP_Servo_ClampCommand((int32_t)speed_percent);

    /* 零指令必须输出精确标定的停车中位，绝不能在零点添加死区补偿。 */
    if (command == 0)
    {
        return neutral_compare;
    }

    if (command > 0)
    {
        direction = 1;
        magnitude = command;
    }
    else
    {
        direction = -1;
        magnitude = -command;
    }

    effective_range = (float)(BSP_SERVO_MAX_OUTPUT_AMP -
                              BSP_SERVO_DEAD_ZONE);
    linear_part = effective_range *
                  ((float)magnitude / (float)BSP_SERVO_SPEED_FULL_SCALE);

    /* 先对线性映射结果四舍五入，再叠加固定的死区补偿量。 */
    compensated_offset = (int32_t)(linear_part + 0.5F) +
                         (int32_t)BSP_SERVO_DEAD_ZONE;
    compare = (int32_t)neutral_compare + direction * compensated_offset;

    minimum_compare = (int32_t)neutral_compare -
                      (int32_t)BSP_SERVO_MAX_OUTPUT_AMP;
    maximum_compare = (int32_t)neutral_compare +
                      (int32_t)BSP_SERVO_MAX_OUTPUT_AMP;

    if (compare < minimum_compare)
    {
        compare = minimum_compare;
    }
    else if (compare > maximum_compare)
    {
        compare = maximum_compare;
    }

    return (uint16_t)compare;
}

void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare)
{
    uint32_t timer_arr;

    /* 本封装函数只允许操作已经配置好的四个电机 PWM 通道。 */
    if ((channel != BSP_SERVO_FR_PWM_CHANNEL) &&
        (channel != BSP_SERVO_FL_PWM_CHANNEL) &&
        (channel != BSP_SERVO_RL_PWM_CHANNEL) &&
        (channel != BSP_SERVO_RR_PWM_CHANNEL))
    {
        return;
    }

    /* 即使应用层直接调用本封装，CCR 也绝对不能超过定时器 ARR。 */
    timer_arr = __HAL_TIM_GET_AUTORELOAD(&BSP_SERVO_TIMER_HANDLE);
    if ((uint32_t)compare > timer_arr)
    {
        compare = (uint16_t)timer_arr;
    }

    __HAL_TIM_SET_COMPARE(&BSP_SERVO_TIMER_HANDLE, channel, compare);
}

void BSP_Servo_SetFrontRightWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_FR_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_FR_NEUTRAL_CCR));
}

void BSP_Servo_SetFrontLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_FL_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_FL_NEUTRAL_CCR));
}

void BSP_Servo_SetRearLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_RL_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_RL_NEUTRAL_CCR));
}

void BSP_Servo_SetRearRightWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_RR_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_RR_NEUTRAL_CCR));
}

void BSP_Chassis_Drive(int16_t throttle, int16_t steering)
{
    int32_t limited_throttle;
    int32_t limited_steering;
    float left_command;
    float right_command;
    float compensation_k;
    float peak_magnitude;
    float normalization_scale;
    int16_t left_output;
    int16_t right_output;

    limited_throttle = BSP_Servo_ClampCommand((int32_t)throttle);
    limited_steering = BSP_Servo_ClampCommand((int32_t)steering);

    /*
     * 基础滑移转向混控公式：
     *
     *   L0 = throttle + steering
     *   R0 = throttle - steering
     *
     * 只有油门输入时，左右两侧速度相同；只有转向输入时，左右两侧速度
     * 大小相等、方向相反，从而实现原地旋转。
     */
    left_command = (float)(limited_throttle + limited_steering);
    right_command = (float)(limited_throttle - limited_steering);

    /* 机械尺寸无效时退回 k=1，保证至少还能使用标准滑移转向混控。 */
    if ((BSP_CHASSIS_TRACK_WIDTH_CM > 0.0F) &&
        (BSP_CHASSIS_WHEEL_BASE_CM > 0.0F))
    {
        compensation_k = BSP_CHASSIS_WHEEL_BASE_CM /
                         BSP_CHASSIS_TRACK_WIDTH_CM;
    }
    else
    {
        compensation_k = 1.0F;
    }

    /*
     * 只有底盘同时存在平移和转向指令时才进行机械补偿。
     * 直线运动和原地旋转不存在需要额外降速的内侧轮，因此不进行补偿。
     *
     * throttle * steering 的符号能够同时在前进和后退状态下判断内侧轮：
     *
     *   throttle * steering > 0  -> 右侧为内侧轮：R = k * R0
     *   throttle * steering < 0  -> 左侧为内侧轮：L = k * L0
     *   throttle * steering = 0  -> 不进行补偿：L = L0，R = R0
     *
     * 这种处理既保留了基础混控关系，也避免原地旋转时出现不对称补偿。
     */
    if ((limited_throttle != 0) && (limited_steering != 0))
    {
        if ((limited_throttle > 0) == (limited_steering > 0))
        {
            right_command *= compensation_k;
        }
        else
        {
            left_command *= compensation_k;
        }
    }

    /*
     * 按比例进行输出限幅：
     *
     *   peak  = max(|L|, |R|)
     *   当 peak > 10000 时，scale = 10000 / peak
     *
     * 左右两侧同时乘以相同系数，可以保持原有转向比例。
     * 如果分别进行简单截断，较大的一侧会单独被削平，导致实际转弯半径
     * 偏离输入指令所表达的转弯半径。
     */
    peak_magnitude = BSP_Servo_AbsFloat(left_command);
    if (BSP_Servo_AbsFloat(right_command) > peak_magnitude)
    {
        peak_magnitude = BSP_Servo_AbsFloat(right_command);
    }

    if (peak_magnitude > (float)BSP_SERVO_SPEED_FULL_SCALE)
    {
        normalization_scale = (float)BSP_SERVO_SPEED_FULL_SCALE /
                              peak_magnitude;
        left_command *= normalization_scale;
        right_command *= normalization_scale;
    }

    left_output = BSP_Servo_FloatToCommand(left_command);
    right_output = BSP_Servo_FloatToCommand(right_command);

    /* 同一侧的前后两个车轮使用完全相同的混控速度指令。 */
    BSP_Servo_SetFrontLeftWheelSpeed(left_output);
    BSP_Servo_SetRearLeftWheelSpeed(left_output);
    BSP_Servo_SetFrontRightWheelSpeed(right_output);
    BSP_Servo_SetRearRightWheelSpeed(right_output);
}

void BSP_Servo_Init(void)
{
    HAL_StatusTypeDef start_status;

    /*
     * 启动 PWM 输出前，先写入四个车轮各自标定后的停车中位值。
     * 这样生成的第一个 PWM 脉冲就是停车指令，不会继承 CubeMX 的初始 CCR
     * 或上一次运行遗留的旧数值，避免上电瞬间车轮意外转动。
     */
    BSP_Servo_SetFrontRightWheelSpeed(0);
    BSP_Servo_SetFrontLeftWheelSpeed(0);
    BSP_Servo_SetRearLeftWheelSpeed(0);
    BSP_Servo_SetRearRightWheelSpeed(0);

    start_status = HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                                     BSP_SERVO_FR_PWM_CHANNEL);
    if (start_status == HAL_OK)
    {
        start_status = HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                                         BSP_SERVO_FL_PWM_CHANNEL);
    }
    if (start_status == HAL_OK)
    {
        start_status = HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                                         BSP_SERVO_RL_PWM_CHANNEL);
    }
    if (start_status == HAL_OK)
    {
        start_status = HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                                         BSP_SERVO_RR_PWM_CHANNEL);
    }

    /* 只启动部分通道并不安全；此时已启动的通道仍保持在停车中位。 */
    if (start_status != HAL_OK)
    {
        Error_Handler();
    }
}
