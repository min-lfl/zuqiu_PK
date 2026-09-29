#include "stm32f10x.h"   // Device header
#include <stdarg.h>
#include "PWM.h"
#include "beep.h"
#include "bsp_advance_tim.h"

void  E49_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA ,ENABLE );
    GPIO_InitStructure .GPIO_Mode =GPIO_Mode_Out_PP ;
    GPIO_InitStructure .GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7;
    GPIO_InitStructure .GPIO_Speed = GPIO_Speed_50MHz ;
    GPIO_Init (GPIOA ,&GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOA ,GPIO_Pin_6 |GPIO_Pin_7);
}

void uart_init(u32 bound)
{
  //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);//使能USART1，GPIOA时钟
	
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9????
   //GPIO_Mode_AF_PP
  //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		
   //USART 初始化设置

		USART_InitStructure.USART_BaudRate =bound;//串口波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx ;	//收发模式
	  //USART_Mode_T
    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1 
}	

char usar_1;
extern u8 djdj;
extern uint16_t tim_mnu;
u8 mun_1;

void USART1_IRQHandler(void)//串口  1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
	  usar_1 = USART_ReceiveData(USART1);//读取接收到的数据
//		USART_SendData(USART1,usar_1);
	
		switch(usar_1)
		{
			case 0X53: mun_1=1;break;//S
			case 0X58: mun_1=2;break;//X
			case 0X57: mun_1=3;break;//W
			case 0X4B: mun_1=4;break;//W
			case 0X4F: mun_1=5;break;//O
			case 0X46: mun_1=6;break;//f1
		}
		if(mun_1==1&&usar_1==0X41)
		{
			zuozhuan();//左转

		}
		if(mun_1==2&&usar_1==0X4D)
		{
			yuozhuan();//右转
		}
		if(mun_1==3&&usar_1==0X51)
		{
			qianjin();//前进
		}
		if(mun_1==2&&usar_1==0X5A)
		{
			houtui();//后退
		}
		
		if(mun_1==6&&usar_1==0X31)
		{
				djdj=3;//停止
		}
		
		if(mun_1==5&&usar_1==0X50)
		{
			djdj=2;
		}
		if(mun_1==4&&usar_1==0X4C)
		{
			djdj=1;
		}
		tim_mnu=0;
	}
//		USART_ClearFlag(USART1,USART_IT_RXNE);
}
