#include "BSP_Servo.h"



//##################################################
//#############函数声明区域#########################
//##################################################
void BSP_Chassis_Drive(int16_t throttle, int16_t steering);
void BSP_Chassis_RampDrive(int16_t throttle, int16_t steering, uint16_t acceleration);
static uint16_t BSP_Servo_SpeedToCompare(int16_t speed_percent,
                                         uint16_t neutral_compare,
                                         int8_t forward_polarity);
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




//让当前值以不超过max_step的步长接近目标值,用于实现线性加减速斜坡
static float BSP_Chassis_ApproachTarget(float current,
                                        float target,
                                        float max_step)
{
    float difference;

    difference = target - current;

    if (difference > max_step)
    {
        return current + max_step;
    }

    if (difference < -max_step)
    {
        return current - max_step;
    }

    return target;
}


//###################################################
//##############电机控制算法区域####################
//###################################################

/**
  * @brief		带时间基准的底盘加减速包装接口
  * @note		本函数只负责产生平滑的油门和转向值,最终仍调用BSP_Chassis_Drive输出
  *				必须在主循环中持续、快速调用,不能只在收到一包遥控数据时调用
  *				算法使用HAL_GetTick计算真实时间,不依赖遥控数据间隔,不需要定时器中断
  * @param		throttle: 目标油门,正数前进、负数后退,范围[-10000,10000]
  * @param		steering: 目标转向,正数右转、负数左转,范围[-10000,10000]
  * @param		acceleration: 每秒允许变化的最大速度指令值
  *				例如20000表示约0.5秒从0变化到10000；传0表示关闭斜坡并立即跟随目标
  * @retval		无
  */
void BSP_Chassis_RampDrive(int16_t throttle,
                           int16_t steering,
                           uint16_t acceleration)
{
    static float current_throttle = 0.0F;
    static float current_steering = 0.0F;
    static uint32_t last_update_tick = 0U;
    static uint8_t ramp_initialized = 0U;

    uint32_t current_tick;
    uint32_t elapsed_ms;
    int32_t target_throttle;
    int32_t target_steering;
    float maximum_change;

    //目标值先限幅,防止错误输入污染斜坡内部状态
    target_throttle = BSP_Servo_ClampCommand((int32_t)throttle);
    target_steering = BSP_Servo_ClampCommand((int32_t)steering);
    current_tick = HAL_GetTick();

    //第一次进入时从停车状态开始计时,不允许第一帧指令直接跳到目标速度
    if (ramp_initialized == 0U)
    {
        ramp_initialized = 1U;
        last_update_tick = current_tick;

        //第一次调用就传0加速度时也必须遵守“立即跟随目标”的接口约定
        if (acceleration == 0U)
        {
            current_throttle = (float)target_throttle;
            current_steering = (float)target_steering;
            BSP_Chassis_Drive((int16_t)target_throttle,
                              (int16_t)target_steering);
            return;
        }

        current_throttle = 0.0F;
        current_steering = 0.0F;
        BSP_Chassis_Drive(0, 0);
        return;
    }

    //加速度为0时视为关闭斜坡,同步内部状态并立即输出目标值
    if (acceleration == 0U)
    {
        current_throttle = (float)target_throttle;
        current_steering = (float)target_steering;
        last_update_tick = current_tick;
        BSP_Chassis_Drive((int16_t)target_throttle,
                          (int16_t)target_steering);
        return;
    }

    //无符号减法可以正确处理HAL_GetTick约49.7天一次的回绕
    elapsed_ms = (uint32_t)(current_tick - last_update_tick);
    if (elapsed_ms < BSP_CHASSIS_RAMP_UPDATE_PERIOD_MS)
    {
        return;
    }

    last_update_tick = current_tick;

    //主循环如果偶然阻塞,只采用有限的dt,避免恢复运行时产生大幅速度跳变
    if (elapsed_ms > BSP_CHASSIS_RAMP_MAX_DT_MS)
    {
        elapsed_ms = BSP_CHASSIS_RAMP_MAX_DT_MS;
    }

    //maximum_change = 加速度(指令/秒) * 实际经过时间(秒)
    maximum_change = (float)acceleration * ((float)elapsed_ms / 1000.0F);

    current_throttle = BSP_Chassis_ApproachTarget(
        current_throttle,
        (float)target_throttle,
        maximum_change);
    current_steering = BSP_Chassis_ApproachTarget(
        current_steering,
        (float)target_steering,
        maximum_change);

    BSP_Chassis_Drive(BSP_Servo_FloatToCommand(current_throttle),
                      BSP_Servo_FloatToCommand(current_steering));
}

/**
  * @brief		四轮滑移转向混控接口,适用于四轮差速小车,这个函数是用户函数,用户可以直接通过这个函数控制小车运动
  * @note			
  * @param		throttle：油门值,			 正数前进、负数后退.有效范围 [-10000, 10000]。
  * @param		steering：左右转速度值,正数右转、负数左转.有效范围 [-10000, 10000]。
	* @retval		无
	*/
void BSP_Chassis_Drive(int16_t throttle, int16_t steering)
{
    int32_t limited_throttle;    //限幅后的油门值
    int32_t limited_steering;    //限幅后的转向值
    int32_t turn_relation;       //油门与转向的符号关系，用于判断内侧轮
    float left_command;          //左侧两轮的逻辑速度
    float right_command;         //右侧两轮的逻辑速度
    float compensation_k;        //内侧轮衰减系数
    float peak_magnitude;
    float normalization_scale;
    int16_t left_output;
    int16_t right_output;

    //第一参数始终是油门，第二参数始终是转向；先分别进行输入限幅
    limited_throttle = BSP_Servo_ClampCommand((int32_t)throttle);
    limited_steering = BSP_Servo_ClampCommand((int32_t)steering);

    /*
     * 基础滑移转向混控公式：
     *
     *   L0 = throttle + steering
     *   R0 = throttle - steering
     *
     * 由此可以直接验证接口语义：
     *   throttle > 0, steering = 0：L0、R0 都为正，整车前进
     *   throttle < 0, steering = 0：L0、R0 都为负，整车后退
     *   throttle = 0, steering > 0：L0 正、R0 负，整车原地右转
     *   throttle = 0, steering < 0：L0 负、R0 正，整车原地左转
     *
     * 这里计算的是已经统一物理方向后的“逻辑轮速”。左右电机实际需要的
     * CCR 增减方向，由四个单轮控制函数中的 FORWARD_POLARITY 宏处理。
     */
    left_command = (float)(limited_throttle + limited_steering);
    right_command = (float)(limited_throttle - limited_steering);

    //计算 k = 轴距 / 轮距；机械尺寸无效时退回 k=1，不进行额外补偿
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
     * compensation_k 被用作“内侧轮衰减系数”，因此有效值必须在 (0,1]。
     * 如果轴距大于轮距导致原始比值超过 1，直接限制为 1，避免错误地
     * 放大内侧轮速度。当前轴距和轮距相等时 k=1，相当于不额外补偿。
     */
    if (compensation_k > 1.0F)
    {
        compensation_k = 1.0F;
    }

    /*
     * 内侧轮选择可以由 throttle * steering 的符号直接推导：
     *
     *   T*S > 0：前进右转或倒车左转，右侧为内侧轮，R = R0*k
     *   T*S < 0：前进左转或倒车右转，左侧为内侧轮，L = L0*k
     *   T*S = 0：纯直行或原地转向，不进行内侧轮补偿
     *
     * 这与上面的 L0=T+S、R0=T-S 完全一致，不需要交换加减号。
     */
    turn_relation = limited_throttle * limited_steering;
    if (turn_relation > 0)
    {
        right_command *= compensation_k;
    }
    else if (turn_relation < 0)
    {
        left_command *= compensation_k;
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

    //浮点转整数，并再次进行最终限幅
    left_output = BSP_Servo_FloatToCommand(left_command);
    right_output = BSP_Servo_FloatToCommand(right_command);

    //严格按照用户逐轮实测的映射：左侧 FL/RL，右侧 FR/RR
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
	* @param	forward_polarity: 该车轮物理前进对应的 CCR 增减极性，只允许 +1 或 -1
	* @retval		最终可以输出给每个轮子的CCR值
	*/
static uint16_t BSP_Servo_SpeedToCompare(int16_t speed_percent,
                                         uint16_t neutral_compare,
                                         int8_t forward_polarity)
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
		//逻辑方向乘以单轮物理极性后，才是该通道真正需要的 CCR 增减方向
    compare = (int32_t)neutral_compare +
              direction * (int32_t)forward_polarity * compensated_offset;

		
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
        BSP_Servo_SpeedToCompare(speed_percent,
                                 BSP_SERVO_FR_NEUTRAL_CCR,
                                 BSP_SERVO_FR_FORWARD_POLARITY));
}

//写入通道2的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetFrontLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_FL_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent,
                                 BSP_SERVO_FL_NEUTRAL_CCR,
                                 BSP_SERVO_FL_FORWARD_POLARITY));
}

//写入通道3的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetRearLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_RL_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent,
                                 BSP_SERVO_RL_NEUTRAL_CCR,
                                 BSP_SERVO_RL_FORWARD_POLARITY));
}

//写入通道4的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Servo_SetRearRightWheelSpeed(int16_t speed_percent)
{
    BSP_Servo_SetPWMCompare(
        BSP_SERVO_RR_PWM_CHANNEL,
        BSP_Servo_SpeedToCompare(speed_percent,
                                 BSP_SERVO_RR_NEUTRAL_CCR,
                                 BSP_SERVO_RR_FORWARD_POLARITY));
}
