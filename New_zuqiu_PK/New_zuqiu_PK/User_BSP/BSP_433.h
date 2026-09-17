#ifndef __BSP_433_H__
#define __BSP_433_H__

#include <stdbool.h>
#include <stdint.h>

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
 * 当前遥控器约每 120~160 ms 重发一次，300 ms 可以容忍一次轻微抖动，
 * 同时又不会让释放后的状态保持太久。若以后更换遥控器，只需调整此宏。
 */
#ifndef BSP_433_KEY_TIMEOUT_MS
#define BSP_433_KEY_TIMEOUT_MS    (300U)
#endif

/**
 * @brief 配置 433 模块，并启动 USART1 的非阻塞接收。
 * @note  必须在 MX_GPIO_Init()、MX_DMA_Init() 和 MX_USART1_UART_Init()
 *        之后调用一次。
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

/* 以下两个函数仅用于 433 模块和链路调试。 */
void Red_uart_433(void);
void Witch_uart_433(void);

#endif /* __BSP_433_H__ */
