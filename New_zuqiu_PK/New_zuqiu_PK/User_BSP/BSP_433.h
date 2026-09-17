#ifndef __BSP_433_H__
#define __BSP_433_H__

//##########头文件引用区########
#include "main.h"
#include "usart.h"

//#########宏定义引用区########



void Set_uart_433_Init(void);		//初始化设置函数


void Red_uart_433(void);			// 读取参数函数
void Witch_uart_433(void);			// 发送报文函数

#endif
