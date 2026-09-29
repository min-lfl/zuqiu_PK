#include<reg51.h>  //包含51单片机寄存器定义的头文件
#include "intrins.h"
#define uint unsigned int 
#define uchar unsigned char 

#define true  1 
#define false 0 
	


sbit trig=P2^0;//触发端口
sbit echo=P3^3;//信号接收端口
sbit tes = P1^0;//测试端口
sbit P1_2 = P1^2;//舵机控制端口
sbit P1_3 = P1^3;//舵机控制端口

uchar time_h,time_l;//时间值
uchar distance_h,distance_l;//距离值
uint time,distance;//时间总值，距离总值
uint d;
uchar d0,d1,d2,d3;
//串口初始化
void init_9600(void)
{
		TMOD = 0x20;			
		TH1 = 0xFD;		
		TL1 = 0xFD;
		SCON = 0x50;			
		PCON &= 0xef;		
		TR1 = 1;				
	
		IE = 0x0;		

}
//发送一个字节
void send_char(unsigned char txd)
{
		SBUF = txd;
		while(!TI);			
		TI = 0;					
}
void send_one_byte_to_char(unsigned char ch)
{
	if ((ch & 0xf0) > 0x90)
	{
		send_char(((ch>>4)&0x0f) + 'A'-0x0a);
	}
	else
	{
		send_char(((ch>>4)&0x0f) + '0'-0x00);
	}
	if ((ch & 0x0f) > 0x09)
	{
		send_char((ch&0x0f) + 'A'-0x0a);
	}
	else
	{
		send_char((ch&0x0f) + '0'-0x00);
	}
}
void send_two_byte_to_char(unsigned int ch)
{
	//3
	if ((ch & 0xf000) > 0x9000)
	{
		send_char(((ch>>12)&0x000f) + 'A'-0x0a);
	}
	else
	{
		send_char(((ch>>12)&0x000f) + '0'-0x00);
	}
	//2
	if ((ch & 0x0f00) > 0x0900)
	{
		send_char(((ch>>8)&0x000f) + 'A'-0x0a);
	}
	else
	{
		send_char(((ch>>8)&0x000f) + '0'-0x00);
	}
	//1
	if ((ch & 0x00f0) > 0x0090)
	{
		send_char(((ch>>4)&0x000f) + 'A'-0x0a);
	}
	else
	{
		send_char(((ch>>4)&0x000f) + '0'-0x00);
	}
	//0
	if ((ch & 0x000f) > 0x0009)
	{
		send_char((ch&0x000f) + 'A'-0x0a);
	}
	else
	{
		send_char((ch&0x000f) + '0'-0x00);
	}
}
void send_four_byte_to_char(uchar a,uchar b,uchar c,uchar d)
{
		send_char(a+'0'-0x00);
		send_char(b+'0'-0x00);
		send_char(c+'0'-0x00);
		send_char(d+'0'-0x00);
}
void send_string(char *p)
{
	char *pchar;

	pchar = p;
	while (*pchar != '\0')
	{
		send_char(*pchar++);		
	}
}
//定时器0初始化
void T0_INIT(void)
{
		TMOD &= 0xF0; /* Timer 0 mode 1 with software gate */ 
		TMOD |= 0x01; /* GATE0=0; C/T0#=0; M10=0; M00=1; */ 
		TH0 = 0x00; /* init values */ 
		TL0 = 0x00; 
		ET0=1; /* enable timer0 interrupt */ 
		//EA=1; /* enable interrupts */ 
		//TR0=1; /* timer0 run */
}
//外部中断1初始化
void INT1_INIT(void)
{
	
	IT1=1;//负跳变中断产生
	EX1=1;//启动外部1中断
	//EA=1;
}
 void delay_nus(unsigned int i)  
{ 
  i=i/10;
  while(--i);
}   

void delay_nms(unsigned int n)  
{ 
  n=n+1;
  while(--n)  
  delay_nus(900);         
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

//主函数
void main(void)
{
		init_9600();
		T0_INIT();
		INT1_INIT();
		trig = 0;
		//tes=0;
		//EA = 1;
		delaynms(1000);
	
		while(1)
		{		
				TH0 = 0x00; 
				TL0 = 0x00; 
				trig = 1;
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				_nop_();
				trig = 0;
			
				while(0 == echo);
				TR0=1;
				while(1 == echo);
				TR0=0;
				time_h = TH0;
				time_l = TL0;
				//send_char(time_h);
				//send_char(time_l);
				time = time_h*256+time_l;
				distance =(int)(time*0.17);
				d0 = distance%10;
				d  = distance/10;
				d1 = d%10;
				d  = d/10;
				d2 = d%10;
				d = d/10;
				d3 = d%10;	
				send_string("L = ");
				send_four_byte_to_char(d3,d2,d1,d0);
				send_string("mm.\r\n");
				//以下增加对舵机的控制，可通过超声波测得的距离来判断行走与否
				if(distance>200){
						P1_3=1;
						delay_nus(1650);
						P1_3=0;
							 
						P1_2=1;
						delay_nus(1350);
						P1_2=0;
						delay_nms(20);
				}
				if(distance<150){ 
						P1_3=1;
						delay_nus(1450);
						P1_3=0;
				 
						P1_2=1;
						delay_nus(1550);
						P1_2=0;
						delay_nms(20);
				}
				
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