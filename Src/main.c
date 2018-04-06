#include "stm32f0xx.h"
#include "vl53l0x_driver.h"
#include "I2C.h"
#include "led.h"
#include "Usart.h"

int main(void)
{
	LED_Init();
	Usart_Init();
	I2C_DevInit();
	VL53L0XInit();
	
	while(1)
	{
		LED_ON;
		LED_OFF;
	}
}



