#include "BSP_servo.H"

//初始化函数,用于初始化四个舵机的通道
void BSP_Servo_Init(void){
	  HAL_StatusTypeDef start_status=HAL_OK;

		//启动 PWM 输出前，先写入四个舵机各自标定后的上电初始位置。
		BSP_Servo_SetJ1Pulse(0);	//写入通道1的ccr值,控制关节1
		BSP_Servo_SetJ2Pulse(0);	//写入通道2的ccr值,控制关节2
		BSP_Servo_SetJ3Pulse(0);	//写入通道3的ccr值,控制关节3
		BSP_Servo_SetGripperPulse(0);	//写入通道4的ccr值,控制关夹爪
	
		//启动定时器四个通道
    if ((HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_Servo_J1_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_Servo_J2_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_Servo_J3_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_Servo_Gripper_PWM_CHANNEL) != HAL_OK))
    {start_status = HAL_ERROR;}

    //如果发生启动失败的,进入异常处理函数
    if (start_status != HAL_OK)
    {
        Error_Handler();
    }
}


/**
	* @brief		接收两个参数,一个是通道选择,一个是ccr数值,最终调用底层函数写入ccr的步骤
	* @note			这个函数每个控制循环里会被调用四次,每次写入其中一个关节或者夹爪的ccr
							并且带有ccr限幅保护功能
	* @param		通道的宏定义,用于选择通道
	* @param		该通道需要写入的具体ccr
	* @retval		无
	*/
void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare)
{
    uint32_t timer_arr;

		/* 函数只允许操作已经配置好的四个舵机 PWM 通道,防止被错误调用导致的越界问题。 */
		//如果发现通道值不对直接返回
    if ((channel != BSP_Servo_J1_PWM_CHANNEL) &&
        (channel != BSP_Servo_J2_PWM_CHANNEL) &&
        (channel != BSP_Servo_J3_PWM_CHANNEL) &&
        (channel != BSP_Servo_Gripper_PWM_CHANNEL))
    {
        return;
    }

    /* 即使应用层直接调用本封装，CCR 也绝对不能超过定时器 ARR。 */
    timer_arr = __HAL_TIM_GET_AUTORELOAD(&BSP_SERVO_TIMER_HANDLE);		//获取当前设定的arr值
		
		//如果大于,那就等于(限幅保护函数)
    if ((uint32_t)compare > timer_arr)			
    {
        compare = (uint16_t)timer_arr;
    }
		
		//最终写入指定的通道
    __HAL_TIM_SET_COMPARE(&BSP_SERVO_TIMER_HANDLE, channel, compare);
}

//写入通道1的ccr值,控制关节1
void BSP_Servo_SetJ1Pulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(
        BSP_Servo_J1_PWM_CHANNEL,
        pulse_val);
}

//写入通道2的ccr值,控制关节2
void BSP_Servo_SetJ2Pulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(
        BSP_Servo_J2_PWM_CHANNEL,
        pulse_val);
}

//写入通道3的ccr值,控制关节3
void BSP_Servo_SetJ3Pulse(uint16_t pulse_val)
{
    BSP_Servo_SetPWMCompare(
        BSP_Servo_J3_PWM_CHANNEL,
        pulse_val);
}

//写入通道4的ccr值,控制夹爪
void BSP_Servo_SetGripperPulse(uint16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_Servo_Gripper_PWM_CHANNEL,
        speed_percent);
}


