#include "stm32f0xx.h"
#include "myi2c.h"
#include "vl53l0x_driver.h"

VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
uint8_t ppdata[12];

int main(void)
{
	I2C_GPIO_Config();
//	VL53L0XInit();
	uint8_t i = 0;
	while(1)
	{
//		VL53L0X_GetRangingMeasurementData(&MyDevice, &RangingMeasurementData);
		ppdata[0] = Single_Read(0x52, 0x00);	
		Single_Write(0x52, 0x00, i++);
	}
}



