#ifndef __USART_H
#define __USART_H

#include "sys.h"

#define USART_REC_LEN   200
#define EN_USART1_RX    1

void uart_init(u32 bound);
void USART2_Init(u32 baud);

#endif
