#include "Usart.h"

void Usart_Init(void)
{
   	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	/*Configure GPIO pin */
	
	GPIO_InitStructure.GPIO_Pin = USART_TX_pin | USART_RX_pin;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_GPIO, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(USART_GPIO, USART_TX_PinSource, GPIO_AF_1);    //Tx
	GPIO_PinAFConfig(USART_GPIO, USART_RX_PinSource, GPIO_AF_1);    //Rx
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitStructure.USART_BaudRate = BAUD_RATE;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

}

