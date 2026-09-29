#ifndef __BSP_ADVANCE_TIM10241_H
#define __BSP_ADVANCE_TIM10241_H

#include "stm32f10x.h"


/************高级定时器TIM参数定义，只限TIM1和TIM8************/
// 当使用不同的定时器的时候，对应的GPIO是不一样的，这点要注意
// 这里我们使用高级控制定时器TIM1

#define            ADVANCE_TIM                   TIM1
#define            ADVANCE_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            ADVANCE_TIM_CLK               RCC_APB2Periph_TIM1
// PWM 信号的频率 F = TIM_CLK/{(ARR+1)*(PSC+1)}
#define            ADVANCE_TIM_PERIOD            (20000-1)
#define            ADVANCE_TIM_PSC               (72-1)
#define            ADVANCE_TIM_PULSE             0

#define            ADVANCE_TIM_IRQ               TIM1_UP_IRQn
#define            ADVANCE_TIM_IRQHandler        TIM1_UP_IRQHandler

// TIM1 输出比较通道
#define            ADVANCE_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            ADVANCE_TIM_CH1_PORT          GPIOA
#define            ADVANCE_TIM_CH1_PIN9          GPIO_Pin_8
#define            ADVANCE_TIM_CH1_PIN11         GPIO_Pin_9
#define            ADVANCE_TIM_CH1_PIN13         GPIO_Pin_10
#define            ADVANCE_TIM_CH1_PIN14         GPIO_Pin_11


/**************************函数声明********************************/
void ADVANCE_TIM1_Init(void);

void Pwm_Tim1_CH1(uint16_t compioni1);//舵机270度
void Pwm_Tim1_CH4(uint16_t compioni4);



#endif
