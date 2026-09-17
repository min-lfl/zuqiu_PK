#include "BSP_433.H"



//###################################################################
//################模块初始化区(hal_delay阻塞初始化)#####################################
//###################################################################
// 初始化设置函数
void Set_uart_433_Init(void)
{
		HAL_Delay(500);
		static uint8_t UartTxBuf3[] = {0xC0,0xFF,0xFF,0x19,0x3E,0x00}; 
	
    // 发送前：PA6拉低，PA7拉高
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    
		HAL_Delay(500);
    // 计算要发送的数据长度
    uint16_t length = 6;
    
    // 启动DMA发送 (非阻塞)
    HAL_UART_Transmit_DMA(&huart1, UartTxBuf3, length);
		
		HAL_Delay(500);
		
		// 发送后：PA6拉低，都拉低,进入接收模式
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

//###################################################################
//################模块调试函数区#####################################
//###################################################################

// 读取参数函数
// 会发送三个C1,无线模块会返回6个字节
// 6个字节不要求解析,这个函数是调试函数,应该循环发送,
// 并且物理上用ch340转串口模块去碰模块的tx再电脑上用串口助手显示
void Red_uart_433(void)
{
	
		static uint8_t UartTxBuf1[] = {0xC1,0xC1,0xC1}; 
	
    // 发送前：PA6拉低，PA7拉高,进入设置模式
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    
		HAL_Delay(500);
    // 计算要发送的数据长度
    uint16_t length = 3;
    
    // 启动DMA发送 (非阻塞)
    HAL_UART_Transmit_DMA(&huart1, UartTxBuf1, length);
		
		//发送后,恢复双拉低的正常模式
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}


// 发送测试报文函数,应该循环发送
// 如果用ch340转usb模块的rx去碰模块的rx接收到了0xAA,0xBB,0xCC表示芯片正确发送了数据
// 如果有接收模块,可以进一步连接电脑上位机查看接收模块是否接到数据,
// 用于测试数据链路是否正确(都没有这些条件时可以通过闪灯判断)
void Witch_uart_433(void)
{
	
		static uint8_t UartTxBuf2[] = {0xAA,0xBB,0xCC}; 
	
    // 发送前,进入双拉低的正常模式
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    
		HAL_Delay(500);
		
    // 计算要发送的数据长度
    uint16_t length = 3;
    
    // 启动DMA发送 (非阻塞)
    HAL_UART_Transmit_DMA(&huart1, UartTxBuf2, length);
}

