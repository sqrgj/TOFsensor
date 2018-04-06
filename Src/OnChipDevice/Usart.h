
#ifndef __USART_H__
#define __USART_H__

#include "stm32f0xx.h"

#define USART_TX_pin			GPIO_Pin_2
#define USART_RX_pin			GPIO_Pin_3
#define USART_GPIO				GPIOA
#define USART_TX_PinSource		GPIO_PinSource2
#define USART_RX_PinSource		GPIO_PinSource3

#define BAUD_RATE				115200


void Usart_Init(void);

#endif

