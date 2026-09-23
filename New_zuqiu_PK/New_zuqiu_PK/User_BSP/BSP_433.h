#ifndef __BSP_433_H__
#define __BSP_433_H__

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "usart.h"

/*
 * ======================== 硬件接口配置区 ========================
 * 移植到其他串口或引脚时，只需要修改下面三个宏，不需要改 BSP_433.c。
 *
 * M0/M1 宏故意展开为“GPIO 端口, GPIO 引脚”两个实参，因此可以直接写成：
 *     HAL_GPIO_WritePin(BSP_433_M0, GPIO_PIN_RESET);
 * 本工程使用 STM32 HAL，故引脚名是 GPIO_PIN_x，而不是标准库的 GPIO_Pin_x。
 * UART 宏填写句柄变量本身（例如 huart2），不要在宏里添加取地址符 &。
 */
#define BSP_433_UART_HANDLE    huart1
#define BSP_433_M0             GPIOA, GPIO_PIN_6
#define BSP_433_M1             GPIOA, GPIO_PIN_7

/*
 * 遥控器共有 8 个按键，外形大致如下：
 *
 * -------------------------------------
 * |                                   |
 * |       上                 前进     |
 * |   左      右                      |
 * |       下                 后退     |
 * |                                   |
 * |              按键1  按键2         |
 * -------------------------------------
 *
 * 每次按键报文固定为两个字节，下面的宏按照“先收到的字节在高 8 位”
 * 进行拼接。例如串口依次收到 0x53、0x41，对应 CMD_Cross_LEFT。
 */

/* 左侧十字按键。 */
#define CMD_Cross_Up       (0x5751U)
#define CMD_Cross_Down     (0x585AU)
#define CMD_Cross_LEFT     (0x5341U)
#define CMD_Cross_RIGHT    (0x584DU)

/* 右侧纵向排列的两个按键。 */
#define CMD_Forward        (0x4F50U)
#define CMD_Back           (0x4B4CU)

/* 下方横向排列的两个按键。 */
#define CMD_One            (0x4342U)
#define CMD_Two            (0x4944U)

/*
 * 遥控器没有发送“按键释放”报文，而是在按住期间重复发送键码。
 * 某个键超过此时间未再次出现，就认为它已经释放。
 *
 * 当前遥控器约每 120~160 ms 重发一次。这里采用160 ms以缩短松键延迟，
 * 从而提高点按转向的精度；如果持续长按时偶尔被误判为松开，可调到180~200 ms。
 */
#ifndef BSP_433_KEY_TIMEOUT_MS
#define BSP_433_KEY_TIMEOUT_MS    (130U)
#endif

/*
 * 一次性读取成功并清空按键状态后，同一按键在此时间内收到的重复键码不会再次置位。
 * 每个按键独立计时；该值应大于遥控器的单次重发间隔，以免一次点按被重复识别。
 */
#ifndef BSP_433_KEY_RETRIGGER_DELAY_MS
#define BSP_433_KEY_RETRIGGER_DELAY_MS    (300U)
#endif

/**
 * @brief 配置 433 模块，并启动所选串口的非阻塞接收。
 * @note  必须在 GPIO、DMA 以及 BSP_433_UART_HANDLE 对应的串口完成初始化后
 *        调用一次。
 */
void Set_uart_433_Init(void);

/**
 * @brief 查询指定按键当前是否仍处于按下状态。
 * @param key_cmd 使用本文件中的 CMD_xxx 宏，不需要记忆具体键码。
 * @retval true  在超时时间内收到过该按键的有效键码。
 * @retval false 按键已超时释放，或 key_cmd 不是已知键码。
 * @note  每次调用都会处理环形缓冲区中的待解析数据并刷新全部按键状态；
 *        请在主循环或控制任务中周期性调用，不要在中断中调用。
 */
bool BSP_433_GetKeyState(uint16_t key_cmd);

/**
 * @brief 一次性读取指定按键，读取成功后立即清除该按键状态。
 * @param key_cmd 使用本文件中的 CMD_xxx 宏。
 * @retval true  本次读取到有效按键状态，且该状态已被清除。
 * @retval false 当前没有有效按键状态，或 key_cmd 不是已知键码。
 * @note  清除后 BSP_433_KEY_RETRIGGER_DELAY_MS 时间内收到的同按键键码不会再次置位；
 *        各按键的清除时间和屏蔽时间互相独立。请勿在中断中调用。
 */
bool BSP_433_GetKeyStateOnce(uint16_t key_cmd);

/* 以下两个函数仅用于 433 模块和链路调试。 */
void Red_uart_433(void);
void Witch_uart_433(void);

#endif /* __BSP_433_H__ */
