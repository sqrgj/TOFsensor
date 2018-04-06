#include "stm32f0xx.h"
#include "vl53l0x_driver.h"
#include "I2C.h"

int main(void)
{
	I2C_DevInit();
	VL53L0XInit();
	
	while(1)
	{

	}
}



