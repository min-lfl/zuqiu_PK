#ifndef __USART1_H
#define __USART1_H

void USART1_IRQHandler(void);
void E49_GPIO_Config(void);
void uart_init(u32 bound);
extern u8 Res;
extern u8 i,b,c,d,z,x,v,n;
#endif
