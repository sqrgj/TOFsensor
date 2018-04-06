#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f0xx.h"
#include <stdbool.h>

#define I2C_SCL_pin				GPIO_Pin_9	
#define I2C_SDA_pin				GPIO_Pin_10
#define I2C_GPIO				GPIOA
#define I2C_SCL_PinSource		GPIO_PinSource9
#define I2C_SDA_PinSource		GPIO_PinSource10

#define I2C_TIMEOUT		60

void I2C_DevInit(void);
bool I2C_Read(uint8_t DevAddr, uint8_t RegAddr, uint8_t* pBuffer, uint8_t NumByteToRead);
bool I2C_Write(uint8_t DevAddr, uint8_t RegAddr, uint8_t* pBuffer, uint8_t NumByteToWrite);

#endif

