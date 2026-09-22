#include "BSP_Motor.h"



//##################################################
//#############函数声明区域#########################
//##################################################
void BSP_Chassis_Drive(int16_t throttle, int16_t steering);
void BSP_Chassis_RampDrive(int16_t throttle, int16_t steering,
                           uint16_t throttle_acceleration,
                           uint16_t steering_acceleration);
static uint16_t BSP_Motor_SpeedToCompare(int16_t speed_percent,
                                         uint16_t neutral_compare,
                                         int8_t forward_polarity);
void BSP_Motor_Init(void);
void BSP_Motor_SetPWMCompare(uint32_t channel, uint16_t compare);
void BSP_Motor_SetFrontRightWheelSpeed(int16_t speed_percent);
void BSP_Motor_SetFrontLeftWheelSpeed(int16_t speed_percent);
void BSP_Motor_SetRearLeftWheelSpeed(int16_t speed_percent);
void BSP_Motor_SetRearRightWheelSpeed(int16_t speed_percent);




//###################################################
//##################数学工具区域#####################
//###################################################
//限幅函数,这里是限制最大速度值(映射ccr前的)
static int32_t BSP_Motor_ClampCommand(int32_t command)
{
    if (command > BSP_Motor_SPEED_FULL_SCALE)
    {
        return BSP_Motor_SPEED_FULL_SCALE;
    }

    if (command < -BSP_Motor_SPEED_FULL_SCALE)
    {
        return -BSP_Motor_SPEED_FULL_SCALE;
    }

    return command;
}

//浮点数绝对值计算函数
static float BSP_Motor_AbsFloat(float value)
{
    return (value < 0.0F) ? -value : value;
}


//将混控算法后的浮点数转整数格式,四舍五入，并且内置限幅
static int16_t BSP_Motor_FloatToCommand(float command)
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
    return (int16_t)BSP_Motor_ClampCommand(rounded_command);
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
  * @brief		油门与转向使用独立加速度的底盘斜坡控制接口
  * @note		本函数只产生平滑指令,最终仍调用BSP_Chassis_Drive输出
  *				必须在主循环中持续调用,不能只在收到遥控报文时调用
  *				内部使用HAL_GetTick作为时间基准,不依赖遥控报文间隔,无需定时器中断
  *				转向采用“低速精细、长按快速、松开快停”的三阶段斜坡
  * @param		throttle: 目标油门,正数前进、负数后退,范围[-10000,10000]
  * @param		steering: 目标转向,正数右转、负数左转；非0目标值会被限制在
  *				[BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND,
  *				 BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND]对应的正负范围内
  * @param		throttle_acceleration: 油门每秒允许变化的最大指令值,传0表示立即跟随
  * @param		steering_acceleration: 精细转向阶段每秒允许变化的指令值,传0表示立即跟随
  * @retval		无
  */
void BSP_Chassis_RampDrive(int16_t throttle,
                           int16_t steering,
                           uint16_t throttle_acceleration,
                           uint16_t steering_acceleration)
{
    static float current_throttle = 0.0F;
    static float current_steering = 0.0F;
    static uint32_t last_update_tick = 0U;
    static uint8_t ramp_initialized = 0U;

    uint32_t current_tick;
    uint32_t elapsed_ms;
    int32_t target_throttle;
    int32_t target_steering;
    float throttle_maximum_change;
    float steering_maximum_change;
    float effective_steering_acceleration;
    float fast_steering_acceleration;
    float steering_ramp_target;
    float steering_acceleration_ratio;
    float current_steering_magnitude;
    uint8_t steering_is_accelerating;
    uint8_t steering_is_reversing;
    uint8_t minimum_steering_is_allowed;
    uint8_t steering_started_at_minimum;

    //目标值先限幅,防止错误输入污染斜坡内部状态
    target_throttle = BSP_Motor_ClampCommand((int32_t)throttle);
    target_steering = BSP_Motor_ClampCommand((int32_t)steering);

    /*
     * 转向目标单独限幅：
     *   1. 非0目标至少提升到MIN_COMMAND，跨过无法推动底盘的机械静摩擦区；
     *   2. 最大目标限制在MAX_COMMAND，避免长按后进入过快的原地旋转。
     *
     * 这里限制的是斜坡目标，不改变底层BSP_Chassis_Drive的通用输入范围。
     */
    if (target_steering > BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND)
    {
        target_steering = BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND;
    }
    else if (target_steering < -BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND)
    {
        target_steering = -BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND;
    }
    else if ((target_steering > 0) &&
             (target_steering < BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND))
    {
        target_steering = BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND;
    }
    else if ((target_steering < 0) &&
             (target_steering > -BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND))
    {
        target_steering = -BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND;
    }

    current_tick = HAL_GetTick();

    //第一次调用时,启用斜坡的轴从0开始；加速度为0的轴立即跟随目标
    if (ramp_initialized == 0U)
    {
        ramp_initialized = 1U;
        last_update_tick = current_tick;

        current_throttle = (throttle_acceleration == 0U) ?
                           (float)target_throttle : 0.0F;
        current_steering = (steering_acceleration == 0U) ?
                           (float)target_steering : 0.0F;

        BSP_Chassis_Drive(BSP_Motor_FloatToCommand(current_throttle),
                          BSP_Motor_FloatToCommand(current_steering));
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

    //油门轴使用独立的线性加减速度
    if (throttle_acceleration == 0U)
    {
        current_throttle = (float)target_throttle;
    }
    else
    {
        throttle_maximum_change = (float)throttle_acceleration *
                                  ((float)elapsed_ms / 1000.0F);
        current_throttle = BSP_Chassis_ApproachTarget(
            current_throttle,
            (float)target_throttle,
            throttle_maximum_change);
    }

    /*
     * 反向操作必须先回到0，再从0向相反方向重新进入精细加速阶段。
     * 如果直接以快速刹车步长接近相反方向的最终目标，一个10ms周期就可能
     * 从+400跨到-400，从而跳过零点附近的精细区，转向角度也会依赖dt大小。
     */
    steering_is_reversing = 0U;
    if (((current_steering > 0.0F) && (target_steering < 0)) ||
        ((current_steering < 0.0F) && (target_steering > 0)))
    {
        steering_is_reversing = 1U;
    }

    steering_ramp_target = (steering_is_reversing != 0U) ?
                           0.0F : (float)target_steering;

    /*
     * 满足下面任意条件时，允许直接从最小有效转向值起步：
     *   1. 油门目标和当前油门都为0，即纯原地转向；
     *   2. 当前油门已经达到组合转向阈值，并且实际运动方向与目标油门一致。
     *
     * 第二个条件正是“按住前进/后退时还能左右转”的关键。如果组合转向仍然
     * 从0缓慢爬升，当前参数需要很长时间才能越过机械无效区，点按就没有反应。
     * 只有当前油门幅度不小于最小转向值时才跳变，可保证|throttle|>=|steering|，
     * 内侧轮最多短暂停止，不会因为转向指令过大而反向。
     * 刚跳到MIN的这个控制周期不再继续增加转向，防止dt较大时转向又越过油门。
     *
     * 这不是为了突然提速，而是跳过实车完全不动作的低速无效区。
     * 同方向快速松开又重新按下时，如果当前值已经低于MIN，也会重新恢复到
     * 最小有效值，避免再次等待斜坡慢慢爬过无效区。
     */
    minimum_steering_is_allowed = 0U;
    steering_started_at_minimum = 0U;
    if ((target_throttle == 0) && (current_throttle == 0.0F))
    {
        minimum_steering_is_allowed = 1U;
    }
    else if (((target_throttle > 0) &&
              (current_throttle >=
               (float)BSP_CHASSIS_RAMP_COMBINED_MIN_THROTTLE)) ||
             ((target_throttle < 0) &&
              (current_throttle <=
               -(float)BSP_CHASSIS_RAMP_COMBINED_MIN_THROTTLE)))
    {
        minimum_steering_is_allowed = 1U;
    }

    if ((steering_is_reversing == 0U) &&
        (minimum_steering_is_allowed != 0U) &&
        (target_steering != 0) &&
        (BSP_Motor_AbsFloat(current_steering) <
         (float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND))
    {
        current_steering = (target_steering > 0) ?
                           (float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND :
                           -(float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND;
        steering_started_at_minimum = 1U;
    }

    /*
     * “正在增加转向”的判定同时要求：目标非0、方向相同、目标幅度更大。
     * 如果目标为0、目标幅度变小或方向相反，均视为刹停/换向过程。
     *
     * 转向斜坡分为三个阶段：
     *   1. 精细起步：点按时使用调用者传入的基础加速度；
     *   2. 渐进加速：随当前转向幅度连续提高加速度，长按时越来越快；
     *   3. 快速刹停：松开、减速或反向时使用独立减速度，减少停止拖尾。
     *
     * 第1、2阶段之间没有硬阈值和倍率突变，加速度按下面的线性公式变化：
     *
     *   ratio = (|current|-MIN) / (MAX-MIN)，并限制到[0,1]
     *   acceleration = fine + (fast-fine) * ratio
     */
    steering_is_accelerating = 0U;
    if ((steering_is_reversing == 0U) &&
        (target_steering != 0) &&
        ((current_steering == 0.0F) ||
         ((current_steering > 0.0F) && (target_steering > 0)) ||
         ((current_steering < 0.0F) && (target_steering < 0))) &&
        (BSP_Motor_AbsFloat((float)target_steering) >
         BSP_Motor_AbsFloat(current_steering)))
    {
        steering_is_accelerating = 1U;
    }

    if (steering_acceleration == 0U)
    {
        current_steering = (float)target_steering;
    }
    else if (steering_started_at_minimum == 0U)
    {
        if (steering_is_accelerating != 0U)
        {
            current_steering_magnitude =
                BSP_Motor_AbsFloat(current_steering);

            if (current_steering_magnitude <=
                (float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND)
            {
                steering_acceleration_ratio = 0.0F;
            }
            else
            {
                steering_acceleration_ratio =
                    (current_steering_magnitude -
                     (float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND) /
                    ((float)BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND -
                     (float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND);

                if (steering_acceleration_ratio > 1.0F)
                {
                    steering_acceleration_ratio = 1.0F;
                }
            }

            fast_steering_acceleration =
                (float)BSP_CHASSIS_RAMP_STEERING_FAST_ACCEL_PER_SEC;
            if (fast_steering_acceleration <
                (float)steering_acceleration)
            {
                fast_steering_acceleration = (float)steering_acceleration;
            }

            effective_steering_acceleration =
                (float)steering_acceleration +
                (fast_steering_acceleration -
                 (float)steering_acceleration) *
                steering_acceleration_ratio;
        }
        else
        {
            effective_steering_acceleration =
                (float)BSP_CHASSIS_RAMP_STEERING_BRAKE_PER_SEC;
        }

        steering_maximum_change = effective_steering_acceleration *
                                  ((float)elapsed_ms / 1000.0F);
        current_steering = BSP_Chassis_ApproachTarget(
            current_steering,
            steering_ramp_target,
            steering_maximum_change);

        //刹车进入物理无效区后直接回到中位，避免电机持续发热却不能转动车身
        if ((steering_ramp_target == 0.0F) &&
            (BSP_Motor_AbsFloat(current_steering) <
             (float)BSP_CHASSIS_RAMP_STEERING_MIN_COMMAND))
        {
            current_steering = 0.0F;
        }
    }

    BSP_Chassis_Drive(BSP_Motor_FloatToCommand(current_throttle),
                      BSP_Motor_FloatToCommand(current_steering));
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
    float steering_ratio;        //转向量占满量程的比例,范围[0,1]
    float inner_wheel_scale;      //随转向量连续变化的内侧轮衰减系数
    float peak_magnitude;
    float normalization_scale;
    int16_t left_output;
    int16_t right_output;

    //第一参数始终是油门，第二参数始终是转向；先分别进行输入限幅
    limited_throttle = BSP_Motor_ClampCommand((int32_t)throttle);
    limited_steering = BSP_Motor_ClampCommand((int32_t)steering);

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
     * 只有|throttle|>|steering|时,左右轮才保持同向运动,属于正常弧线转弯；
     * 原地旋转或某侧已经反转时不做几何衰减,避免破坏原地旋转的左右对称性。
     *
     * 固定将内侧轮乘以k会产生不连续：steering从0变成1时,内侧轮会立刻
     * 从100%跳到k。为保证轻点方向时只产生轻微差速,改为线性插值：
     *
     *   steering_ratio  = |steering| / STEERING_MAX，并限制到[0,1]
     *   inner_wheel_scale = 1 - (1-k) * steering_ratio
     *
     * steering=0时scale=1,完全不补偿；转向达到本模块允许的MAX_COMMAND时
     * scale才平滑到达k。BSP_Chassis_Drive直接收到更大转向值时ratio仍限制为1。
     * 内侧轮选择可以由 throttle * steering 的符号直接推导：
     *
     *   T*S > 0：前进右转或倒车左转，右侧为内侧轮，R = R0*scale
     *   T*S < 0：前进左转或倒车右转，左侧为内侧轮，L = L0*scale
     *   T*S = 0：纯直行或原地转向，不进行内侧轮补偿
     *
     * 这与上面的 L0=T+S、R0=T-S 完全一致，不需要交换加减号。
     */
    steering_ratio = BSP_Motor_AbsFloat((float)limited_steering) /
                     (float)BSP_CHASSIS_RAMP_STEERING_MAX_COMMAND;
    if (steering_ratio > 1.0F)
    {
        steering_ratio = 1.0F;
    }
    inner_wheel_scale = 1.0F -
                        (1.0F - compensation_k) * steering_ratio;

    turn_relation = limited_throttle * limited_steering;
    if ((BSP_Motor_AbsFloat((float)limited_throttle) >
         BSP_Motor_AbsFloat((float)limited_steering)) &&
        (turn_relation > 0))
    {
        right_command *= inner_wheel_scale;
    }
    else if ((BSP_Motor_AbsFloat((float)limited_throttle) >
              BSP_Motor_AbsFloat((float)limited_steering)) &&
             (turn_relation < 0))
    {
        left_command *= inner_wheel_scale;
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
    peak_magnitude = BSP_Motor_AbsFloat(left_command);
    if (BSP_Motor_AbsFloat(right_command) > peak_magnitude)
    {
        peak_magnitude = BSP_Motor_AbsFloat(right_command);
    }

    if (peak_magnitude > (float)BSP_Motor_SPEED_FULL_SCALE)
    {
        normalization_scale = (float)BSP_Motor_SPEED_FULL_SCALE /
                              peak_magnitude;
        left_command *= normalization_scale;
        right_command *= normalization_scale;
    }

    //浮点转整数，并再次进行最终限幅
    left_output = BSP_Motor_FloatToCommand(left_command);
    right_output = BSP_Motor_FloatToCommand(right_command);

    //严格按照用户逐轮实测的映射：左侧 FL/RL，右侧 FR/RR
    BSP_Motor_SetFrontLeftWheelSpeed(left_output);
    BSP_Motor_SetRearLeftWheelSpeed(left_output);
    BSP_Motor_SetFrontRightWheelSpeed(right_output);
    BSP_Motor_SetRearRightWheelSpeed(right_output);
}


/**
  * @brief	引入死区补偿,并且将单个车轮的速度值映射成CCR值。	
  * @note		死区补偿的思路参考线性死区补偿,并且在这里做的ccr值限幅
  * @param	speed_percent: 上层函数得到的速度值
  * @param	neutral_compare: 电机CCR的零点位置,大概是1500附近,这里取宏定义
	* @param	forward_polarity: 该车轮物理前进对应的 CCR 增减极性，只允许 +1 或 -1
	* @retval		最终可以输出给每个轮子的CCR值
	*/
static uint16_t BSP_Motor_SpeedToCompare(int16_t speed_percent,
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
    command = BSP_Motor_ClampCommand((int32_t)speed_percent);

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
    effective_range = (float)(BSP_Motor_MAX_OUTPUT_AMP -BSP_Motor_DEAD_ZONE);
		//计算线性映射值,公式为可调范围*(目标速度/最大速度)
    linear_part = effective_range *((float)magnitude / (float)BSP_Motor_SPEED_FULL_SCALE);
    //把线性映射值加上死区值,得到死区补偿后的输出结果
    compensated_offset = (int32_t)(linear_part + 0.5F) +(int32_t)BSP_Motor_DEAD_ZONE;
		
		/* #####死开始融合得出最终ccr##### */
		//逻辑方向乘以单轮物理极性后，才是该通道真正需要的 CCR 增减方向
    compare = (int32_t)neutral_compare +
              direction * (int32_t)forward_polarity * compensated_offset;

		
		/* #####限幅处理区域##### */
		//算出允许的最大ccr和最小ccr
    minimum_compare = (int32_t)neutral_compare -
                      (int32_t)BSP_Motor_MAX_OUTPUT_AMP;
    maximum_compare = (int32_t)neutral_compare +
                      (int32_t)BSP_Motor_MAX_OUTPUT_AMP;
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
void BSP_Motor_Init(void)	//初始化函数
{
    HAL_StatusTypeDef start_status=HAL_OK;

		//启动 PWM 输出前，先写入四个车轮各自标定后的停车值。
    BSP_Motor_SetFrontRightWheelSpeed(0);
    BSP_Motor_SetFrontLeftWheelSpeed(0);
    BSP_Motor_SetRearLeftWheelSpeed(0);
    BSP_Motor_SetRearRightWheelSpeed(0);

		//启动定时器四个通道
    if ((HAL_TIM_PWM_Start(&BSP_Motor_TIMER_HANDLE, BSP_Motor_FR_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_Motor_TIMER_HANDLE, BSP_Motor_FL_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_Motor_TIMER_HANDLE, BSP_Motor_RL_PWM_CHANNEL) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&BSP_Motor_TIMER_HANDLE, BSP_Motor_RR_PWM_CHANNEL) != HAL_OK))
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
void BSP_Motor_SetPWMCompare(uint32_t channel, uint16_t compare)
{
    uint32_t timer_arr;

    /* 函数只允许操作已经配置好的四个电机 PWM 通道,防止被错误调用导致的越界问题。 */
		//如果发现通道值不对直接返回
    if ((channel != BSP_Motor_FR_PWM_CHANNEL) &&
        (channel != BSP_Motor_FL_PWM_CHANNEL) &&
        (channel != BSP_Motor_RL_PWM_CHANNEL) &&
        (channel != BSP_Motor_RR_PWM_CHANNEL))
    {
        return;
    }

    /* 即使应用层直接调用本封装，CCR 也绝对不能超过定时器 ARR。 */
    timer_arr = __HAL_TIM_GET_AUTORELOAD(&BSP_Motor_TIMER_HANDLE);		//获取当前设定的arr值
		
		//如果大于,那就等于(限幅保护函数)
    if ((uint32_t)compare > timer_arr)			
    {
        compare = (uint16_t)timer_arr;
    }
		
		//最终写入指定的通道
    __HAL_TIM_SET_COMPARE(&BSP_Motor_TIMER_HANDLE, channel, compare);
}

//写入通道1的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Motor_SetFrontRightWheelSpeed(int16_t speed_percent)
{
    BSP_Motor_SetPWMCompare(
        BSP_Motor_FR_PWM_CHANNEL,
        BSP_Motor_SpeedToCompare(speed_percent,
                                 BSP_Motor_FR_NEUTRAL_CCR,
                                 BSP_Motor_FR_FORWARD_POLARITY));
}

//写入通道2的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Motor_SetFrontLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Motor_SetPWMCompare(
        BSP_Motor_FL_PWM_CHANNEL,
        BSP_Motor_SpeedToCompare(speed_percent,
                                 BSP_Motor_FL_NEUTRAL_CCR,
                                 BSP_Motor_FL_FORWARD_POLARITY));
}

//写入通道3的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Motor_SetRearLeftWheelSpeed(int16_t speed_percent)
{
    BSP_Motor_SetPWMCompare(
        BSP_Motor_RL_PWM_CHANNEL,
        BSP_Motor_SpeedToCompare(speed_percent,
                                 BSP_Motor_RL_NEUTRAL_CCR,
                                 BSP_Motor_RL_FORWARD_POLARITY));
}

//写入通道4的速度值,BSP_Chassis_Drive函数得到或者自己传入
void BSP_Motor_SetRearRightWheelSpeed(int16_t speed_percent)
{
    BSP_Motor_SetPWMCompare(
        BSP_Motor_RR_PWM_CHANNEL,
        BSP_Motor_SpeedToCompare(speed_percent,
                                 BSP_Motor_RR_NEUTRAL_CCR,
                                 BSP_Motor_RR_FORWARD_POLARITY));
}
