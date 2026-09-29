#include "BSP_servo.h"


//##################################################
//#############函数声明区域#########################
//##################################################
static uint16_t BSP_Servo_JointAngleToCompare(float joint_angle_deg,
                                               uint16_t horizontal_compare,
                                               int8_t direction);
static void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare);


//###################################################
//##################数学工具区域#####################
//###################################################
/**
 * @brief  把一个关节的局部转角换算为标定后的 CCR。
 * @param  joint_angle_deg: 相对水平标定姿态的局部关节角，单位为度。
 * @param  horizontal_compare: 该关节处于水平标定姿态时的 CCR。
 * @param  direction: 该关节正角度对应的 CCR 变化方向，只能是 +1 或 -1。
 * @retval 限制在标准舵机脉宽范围内的 CCR。
 */
static uint16_t BSP_Servo_JointAngleToCompare(float joint_angle_deg,
                                               uint16_t horizontal_compare,
                                               int8_t direction)
{
    float compare;

    /*
     * 水平标定值作为零点，方向宏负责吸收三个舵机不同的机械安装方向：
     *
     * CCR = 水平标定 CCR + 方向 * 局部关节角 * 每度 CCR
     */
    compare = (float)horizontal_compare +
              (float)direction * joint_angle_deg *
              BSP_SERVO_CCR_PER_DEGREE;

    //算法输出只允许处于标准舵机的 0.5ms～2.5ms 脉宽范围内。
    if (compare <= (float)BSP_SERVO_MIN_PULSE_CCR)
    {
        return BSP_SERVO_MIN_PULSE_CCR;
    }

    if (compare >= (float)BSP_SERVO_MAX_PULSE_CCR)
    {
        return BSP_SERVO_MAX_PULSE_CCR;
    }

    //浮点数加 0.5 后再转整数，实现正数的四舍五入。
    return (uint16_t)(compare + 0.5F);
}


//###################################################
//##############机械臂绝对角控制区域#################
//###################################################
/**
 * @brief  设置三段活动连杆相对于地面坐标系的绝对角度。
 * @note   地面向前为 +X，竖直向上为 +Y；从 +X 转向 +Y 的角度为正。
 *         本函数只负责姿态角，不根据 AB/BC/CD 长度计算末端位置。
 * @param  ab_angle_deg: AB 连杆的地面绝对角，单位为度。
 * @param  bc_angle_deg: BC 连杆的地面绝对角，单位为度。
 * @param  cd_angle_deg: CD 连杆的地面绝对角，单位为度。
 * @retval 无
 */
void BSP_Servo_SetAbsoluteAngles(float ab_angle_deg,
                                 float bc_angle_deg,
                                 float cd_angle_deg)
{
    float joint1_relative_angle_deg;
    float joint2_relative_angle_deg;
    float joint3_relative_angle_deg;
    uint16_t joint1_compare;
    uint16_t joint2_compare;
    uint16_t joint3_compare;

    /*
     * 舵机控制的是相邻连杆之间的“局部角”，而函数接收的是各连杆的
     * “地面绝对角”。两者的换算关系如下：
     *
     *   A 点关节角 = AB 的绝对角
     *   B 点关节角 = BC 的绝对角 - AB 的绝对角
     *   C 点关节角 = CD 的绝对角 - BC 的绝对角
     *
     * 例如只让 AB 从 0 度抬到 30 度，而 BC、CD 仍保持水平：
     * 三个局部关节角分别为 +30、-30、0 度。B 关节的 -30 度正好
     * 抵消 A 关节对后续连杆方向造成的影响。
     */
    joint1_relative_angle_deg = ab_angle_deg;
    joint2_relative_angle_deg = bc_angle_deg - ab_angle_deg;
    joint3_relative_angle_deg = cd_angle_deg - bc_angle_deg;

    joint1_compare = BSP_Servo_JointAngleToCompare(
        joint1_relative_angle_deg,
        BSP_SERVO_J1_HORIZONTAL_CCR,
        BSP_SERVO_J1_DIRECTION);
    joint2_compare = BSP_Servo_JointAngleToCompare(
        joint2_relative_angle_deg,
        BSP_SERVO_J2_HORIZONTAL_CCR,
        BSP_SERVO_J2_DIRECTION);
    joint3_compare = BSP_Servo_JointAngleToCompare(
        joint3_relative_angle_deg,
        BSP_SERVO_J3_HORIZONTAL_CCR,
        BSP_SERVO_J3_DIRECTION);

    BSP_Servo_SetJ1Pulse(joint1_compare);
    BSP_Servo_SetJ2Pulse(joint2_compare);
    BSP_Servo_SetJ3Pulse(joint3_compare);
}


//###################################################
//##############舵机底层驱动区域#####################
//###################################################
//初始化四个舵机 PWM 通道。
void BSP_Servo_Init(void)
{
    HAL_StatusTypeDef start_status = HAL_OK;

    /*
     * 启动 PWM 前先把四路 CCR 清零，不在初始化过程中命令机械臂突然运动。
     * 初始化完成后，由用户显式调用 BSP_Servo_SetAbsoluteAngles() 设置姿态；
     * 夹爪也继续使用原始 CCR 接口单独控制。
     */
    BSP_Servo_SetJ1Pulse(0U);
    BSP_Servo_SetJ2Pulse(0U);
    BSP_Servo_SetJ3Pulse(0U);
    BSP_Servo_SetGripperPulse(0U);

    //四个通道分别启动，某一路失败也会继续检查并启动其余通道。
    if (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                          BSP_Servo_J1_PWM_CHANNEL) != HAL_OK)
    {
        start_status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                          BSP_Servo_J2_PWM_CHANNEL) != HAL_OK)
    {
        start_status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                          BSP_Servo_J3_PWM_CHANNEL) != HAL_OK)
    {
        start_status = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE,
                          BSP_Servo_Gripper_PWM_CHANNEL) != HAL_OK)
    {
        start_status = HAL_ERROR;
    }

    //任意一路启动失败都进入工程统一的异常处理函数。
    if (start_status != HAL_OK)
    {
        Error_Handler();
    }
}


/**
 * @brief  向指定舵机通道写入原始 CCR。
 * @note   本函数供四个公开的 Pulse 接口共用，只负责通道检查和 ARR 限幅。
 *         关节的 500～2500 安全范围由绝对角换算函数负责；保留原始接口后，
 *         用户仍可直接写 CCR 做标定，也可以给夹爪写入任意测试值。
 * @param  channel: BSP_servo.h 中定义的四个 TIM 通道之一。
 * @param  compare: 需要写入的 CCR。
 * @retval 无
 */
static void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare)
{
    uint32_t timer_arr;

    //只允许操作已经配置好的四个舵机 PWM 通道。
    if ((channel != BSP_Servo_J1_PWM_CHANNEL) &&
        (channel != BSP_Servo_J2_PWM_CHANNEL) &&
        (channel != BSP_Servo_J3_PWM_CHANNEL) &&
        (channel != BSP_Servo_Gripper_PWM_CHANNEL))
    {
        return;
    }

    //即使直接调用原始 CCR 接口，写入值也不能超过当前定时器 ARR。
    timer_arr = __HAL_TIM_GET_AUTORELOAD(&BSP_SERVO_TIMER_HANDLE);
    if ((uint32_t)compare > timer_arr)
    {
        compare = (uint16_t)timer_arr;
    }

    __HAL_TIM_SET_COMPARE(&BSP_SERVO_TIMER_HANDLE, channel, compare);
}


//写入通道1的原始 CCR，主要用于关节1标定和调试。
void BSP_Servo_SetJ1Pulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(BSP_Servo_J1_PWM_CHANNEL, pulse_val);
}

//写入通道2的原始 CCR，主要用于关节2标定和调试。
void BSP_Servo_SetJ2Pulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(BSP_Servo_J2_PWM_CHANNEL, pulse_val);
}

//写入通道3的原始 CCR，主要用于关节3标定和调试。
void BSP_Servo_SetJ3Pulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(BSP_Servo_J3_PWM_CHANNEL, pulse_val);
}

//写入通道4的原始 CCR，夹爪暂时继续由用户直接控制。
void BSP_Servo_SetGripperPulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(BSP_Servo_Gripper_PWM_CHANNEL, pulse_val);
}


