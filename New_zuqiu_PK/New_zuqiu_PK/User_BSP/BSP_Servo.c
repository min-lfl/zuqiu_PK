#include "BSP_Servo.h"



//##################################################
//#############函数声明区域#########################
//##################################################
void BSP_Chassis_Drive(int16_t throttle, int16_t steering);
static uint16_t BSP_Servo_SpeedToCompare(int16_t speed_percent,uint16_t neutral_compare);
void BSP_Servo_Init(void);
void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare);
void BSP_Servo_SetFrontRightWheelSpeed(int16_t speed_percent);
void BSP_Servo_SetFrontLeftWheelSpeed(int16_t speed_percent);
void BSP_Servo_SetRearLeftWheelSpeed(int16_t speed_percent);
void BSP_Servo_SetRearRightWheelSpeed(int16_t speed_percent);




//###################################################
//##################数学工具区域#####################
//###################################################
//限幅函数,这里是限制最大速度值(映射ccr前的)
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

//浮点数绝对值计算函数
static float BSP_Servo_AbsFloat(float value)
{
    return (value < 0.0F) ? -value : value;
}


//将混控算法后的浮点数转整数格式,四舍五入，并且内置限幅
static int16_t BSP_Servo_FloatToCommand(float command)
{
    int32_t rounded_command;

		//判断符号后加减0.5
    if (command >= 0.0F)
    {
        rounded_command = (int32_t)(command + 0.5F);
    }
    else
    {
        rounded_command = (int32_t)(command - 0.5F);
    }

		//限幅后开始强转,强转会直接吧小数点后面的丢掉
    return (int16_t)BSP_Servo_ClampCommand(rounded_command);
}




//###################################################
//##############电机控制算法区域####################
//###################################################

/**
  * @brief		四轮滑移转向混控接口,适用于四轮差速小车,这个函数是用户函数,用户可以直接通过这个函数控制小车运动
  * @note			
  * @param		throttle：油门值,			 正数前进、负数后退.有效范围 [-10000, 10000]。
  * @param		steering：左右转速度值,正数右转、负数左转.有效范围 [-10000, 10000]。
	* @retval		无
	*/
void BSP_Chassis_Drive(int16_t throttle, int16_t steering)
{
	int32_t limited_throttle;		//限幅后的油门缓存区
    int32_t limited_steering; //限幅后的左右转缓存区
    float left_command;				//融合后的左边速度
    float right_command;			//融合后的右边速度
		float compensation_k;			//混动补偿系数缓存区
    float peak_magnitude;
    float normalization_scale;
    int16_t left_output;
    int16_t right_output;

	  //依旧先来个小限幅,防止流口水的用户输入错误的速度
    limited_throttle = BSP_Servo_ClampCommand((int32_t)throttle);
    limited_steering = BSP_Servo_ClampCommand((int32_t)steering);

    /*
     * 基础滑移转向混控公式：这里可以得到数据融合后左边两个轮子的速度和右边两个轮子的速度
     *
     *   L0 = throttle + steering
     *   R0 = throttle - steering
     *
     * 只有油门输入时，左右两侧速度相同；只有转向输入时，左右两侧速度
     * 大小相等、方向相反，从而实现原地旋转。
     */
		//左边两个轮子速度
    left_command = (float)(limited_throttle + limited_steering);
		//右边两个轮子速度
    right_command = (float)(limited_throttle - limited_steering);


		//#############混动算法部分,它是为了解决两边轮子速度差和转向角度非线性的问题###########
    //计算混动系数,机械尺寸无效时退回 k=1，保证至少还能使用标准滑移转向混控。
    if ((BSP_CHASSIS_TRACK_WIDTH_CM > 0.0F) &&	//保证轮距和轴距都大于0
        (BSP_CHASSIS_WHEEL_BASE_CM > 0.0F))
    {
        compensation_k = BSP_CHASSIS_WHEEL_BASE_CM /	//计算轮距和轴距
                         BSP_CHASSIS_TRACK_WIDTH_CM;
    }else{
        compensation_k = 1.0F;
    }
		

		/*
		 * 转向机械补偿：只有在“边走边转”时才对内侧轮进行降速补偿。
		 * 可以得到的是补偿值,要给哪个轮子减速具体多少的值
		 * 原地自转 (throttle == 0) 或 纯直线时，所有分支均不满足，自动跳过。
		 */
		/* 前进情况 */
		if (limited_throttle > 0)
		{
				if (limited_steering > 0)
				{
						//前进 + 右转：右侧为内侧轮，降低右侧车轮输出
						right_command = right_command * compensation_k;
				}
				else if (limited_steering < 0)
				{
						//前进 + 左转：左侧为内侧轮，降低左侧车轮输出
						left_command = left_command * compensation_k;
				}
		}
		/* 后退情况 */
		else if (limited_throttle < 0)
		{
				if (limited_steering > 0)
				{
						//倒车 + 右转：依据差速几何，此时左侧为需补偿轮
						left_command = left_command * compensation_k;
				}
				else if (limited_steering < 0)
				{
						//倒车 + 左转：依据差速几何，此时右侧为需补偿轮
						right_command = right_command * compensation_k;
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

		
		//浮点转整数,并且依旧加点小限幅
    left_output = BSP_Servo_FloatToCommand(left_command);
    right_output = BSP_Servo_FloatToCommand(right_command);

		
		//调用底层的四个轮子驱动函数去写入速度值
    BSP_Servo_SetFrontLeftWheelSpeed(left_output);
    BSP_Servo_SetRearLeftWheelSpeed(left_output);
    BSP_Servo_SetFrontRightWheelSpeed(right_output);
    BSP_Servo_SetRearRightWheelSpeed(right_output);
}


/**
  * @brief	引入死区补偿,并且将单个车轮的速度值映射成CCR值。	
  * @note		死区补偿的思路参考线性死区补偿,并且在这里做的ccr值限幅
  * @param	speed_percent: 上层函数得到的速度值
  * @param	neutral_compare: 电机CCR的零点位置,大概是1500附近,这里取宏定义
	* @retval		最终可以输出给每个轮子的CCR值
	*/
static uint16_t BSP_Servo_SpeedToCompare(int16_t speed_percent,
                                         uint16_t neutral_compare)
{
    int32_t command;		      //缓存限幅后速度
    int32_t magnitude;	      //缓存正反转判断后的速度
    int32_t direction;	      //正反转判断标志位
	  int32_t compensated_offset;	//缓存死区补偿后的输出结果
	  int32_t compare;		      //最终输出ccr缓存区
    int32_t minimum_compare;	//限幅最小
    int32_t maximum_compare;	//限幅最大
	  float effective_range;    //缓存死区可调范围
		float linear_part;		    //缓存死区线性映射值

		//限幅函数
    command = BSP_Servo_ClampCommand((int32_t)speed_percent);

    /* 零指令必须输出精确标定的停车中位，绝不能在零点添加死区补偿。 */
    if (command == 0)		//如果速度是0
    {
        return neutral_compare;
    }

    if (command > 0)		//如果速度是正转
    {
        direction = 1;
        magnitude = command;
    }
    else								//如果速度是反转
    {
        direction = -1;
        magnitude = -command;
    }

		
		/* #####死区补偿算法三板斧##### */
		//输出范围减去死区得到可调范围
    effective_range = (float)(BSP_SERVO_MAX_OUTPUT_AMP -BSP_SERVO_DEAD_ZONE);
		//计算线性映射值,公式为可调范围*(目标速度/最大速度)
    linear_part = effective_range *((float)magnitude / (float)BSP_SERVO_SPEED_FULL_SCALE);
    //把线性映射值加上死区值,得到死区补偿后的输出结果
    compensated_offset = (int32_t)(linear_part + 0.5F) +(int32_t)BSP_SERVO_DEAD_ZONE;
		
		/* #####死开始融合得出最终ccr##### */
		//用0点也是停转的ccr加上刚刚算出的死区线性映射后的ccr值,这里加还是减调了很多次,为什么要做四轮车啊啊啊
    compare = (int32_t)neutral_compare + direction * compensated_offset;

		
		/* #####限幅处理区域##### */
		//算出允许的最大ccr和最小ccr
    minimum_compare = (int32_t)neutral_compare -
                      (int32_t)BSP_SERVO_MAX_OUTPUT_AMP;
    maximum_compare = (int32_t)neutral_compare +
                      (int32_t)BSP_SERVO_MAX_OUTPUT_AMP;
		//依旧限幅
    if (compare < minimum_compare)
    {
        compare = minimum_compare;
    }
    else if (compare > maximum_compare)
    {
        compare = maximum_compare;
    }

		//输出,没啥好说的
    return (uint16_t)compare;
}




//###################################################
//##############电机底层驱动区域##############
//###################################################
void BSP_Servo_Init(void)	//初始化函数
{
    HAL_StatusTypeDef start_status=HAL_OK;

		//启动 PWM 输出前，先写入四个车轮各自标定后的停车值。
    BSP_Servo_SetFrontRightWheelSpeed(0);
    BSP_Servo_SetFrontLeftWheelSpeed(0);
    BSP_Servo_SetRearLeftWheelSpeed(0);
    BSP_Servo_SetRearRightWheelSpeed(0);

		//启动定时器四个通道
    if ((HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_SERVO_FR_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_SERVO_FL_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_SERVO_RL_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_SERVO_TIMER_HANDLE, BSP_SERVO_RR_PWM_CHANNEL) != HAL_OK))
    {start_status = HAL_ERROR;}

    //如果发生启动失败的,进入异常处理函数
    if (start_status != HAL_OK)
    {
        Error_Handler();
    }
}


/**
	* @brief		接收两个参数,一个是通道选择,一个是ccr数值,最终调用底层函数写入ccr的步骤
	* @note			这个函数每个控制循环里会被调用四次,每次写入其中一个轮子的ccr
							并且带有ccr限幅保护功能
	* @param		通道的宏定义,用于选择通道
	* @param		该通道需要写入的具体ccr
	* @retval		无
	*/
void BSP_Servo_SetPWMCompare(uint32_t channel, uint16_t compare)
{
    uint32_t timer_arr;

    /* 函数只允许操作已经配置好的四个电机 PWM 通道,防止被错误调用导致的越界问题。 */
		//如果发现通道值不对直接返回
    if ((channel != BSP_SERVO_FR_PWM_CHANNEL) &&
        (channel != BSP_SERVO_FL_PWM_CHANNEL) &&
        (channel != BSP_SERVO_RL_PWM_CHANNEL) &&
        (channel != BSP_SERVO_RR_PWM_CHANNEL))
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

//写入通道1的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetFrontRightWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_FR_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_FR_NEUTRAL_CCR));
}

//写入通道2的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetFrontLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_FL_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_FL_NEUTRAL_CCR));
}

//写入通道3的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetRearLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_RL_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_RL_NEUTRAL_CCR));
}

//写入通道4的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetRearRightWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_RR_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent, BSP_SERVO_RR_NEUTRAL_CCR));
}




