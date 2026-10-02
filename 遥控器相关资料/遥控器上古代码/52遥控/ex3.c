#include<reg51.h>  //包含51单片机寄存器定义的头文件
#include "intrins.h"
#define u32 unsigned int 
#define u8 unsigned char 
#define u16 unsigned short int
#define true  1 
#define false 0
	
//配置无线模式端口定义	
sbit  WX_M0=P1^1;
sbit  WX_M1=P1^2;	

//方向键
sbit Left    = P0^0;
sbit Right   = P0^1;
sbit Forward = P0^2;
sbit Back    = P0^3;
//加减速键
sbit UpSpeed   = P0^4;
sbit DownSpeed = P0^5;
//功能键
sbit F1 = P0^6;
sbit F2 = P0^7;
sbit F3 = P1^3;
sbit F4 = P1^4;
//拨码开关
sbit S1 = P2^0;
sbit S2 = P2^1;
sbit S3 = P2^2;
sbit S4 = P2^3;
sbit S5 = P2^4;
sbit S6 = P2^5;
sbit S7 = P2^6;
sbit S8 = P2^7;
//串口初始化

void init_9600(void)
{
		TMOD = 0x20;			
		TH1 = 0xFD;		
		TL1 = 0xFD;
		SCON = 0x50;			
		PCON &= 0x7f;		
		TR1 = 1;				
	
		IE = 0x0;
}

//发送一个字节
void send_char(u8 txd)
{
	SBUF =txd;
	while(!TI);			
		TI = 0;								
}
void send_string(u8 *p)
{
	char *pchar;

	pchar = p;
	while (*pchar != '\0')
	{
		send_char(*pchar++);		
	}
}
//延时函数1毫秒
void delay1ms()
{
   unsigned char i,j;	
	 for(i=0;i<10;i++)
	  for(j=0;j<33;j++)
	   ;		 
 }

//延时函数，n毫秒
 void delaynms(unsigned int n)
 {
		unsigned int i;
		for(i=0;i<n;i++)
	  delay1ms();
 }
 
void main(void)
{
	init_9600();
	WX_M0=0;
	WX_M1=0;
	while(1)
	{		
		if(Left==0)//左
		{
			send_string("SA");//ASDF
		}
		if(Right==0)//右
		{
			send_string("XM");//MXCV
		}
		if(Forward==0)//前
		{
			send_string("WQ");//QWER
		}
		if(Back==0)//后
		{
			send_string("XZ");//ZXYJ
		}
		if(UpSpeed==0)
		{
			send_string("OP");//POIU
		}
		if(DownSpeed==0)
		{
			send_string("KL");//LKJH
		}
		if(F1==0)
		{
			send_string("Aa");
		}
		if(F2==0)
		{
			send_string("Bb");
		}
		if(F3==0)
		{
			send_string("Cc");
		}
		if(F4==0)
		{
			send_string("Dd");
		}
		delaynms(48);//60
	}
}

//外部中断1中断服务函数
void it_INT1(void) interrupt 2 
{ 
	IE1 = 0;
}
//定时器0中断服务函数
void it_timer0(void) interrupt 1 
{ 
	TF0 = 0;
}