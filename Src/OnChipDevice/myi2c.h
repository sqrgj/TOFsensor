#ifndef __MYI2C_H__
#define __MYI2C_H__

#include "stm32f0xx.h"

#define SCL_GPIO 		GPIOA
#define SDA_GPIO 		GPIOA
#define SCL_PIN 		GPIO_Pin_9
#define SDA_PIN 		GPIO_Pin_10

#define SCL_H         	SCL_GPIO->BSRR |= SCL_PIN /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         	SCL_GPIO->BRR  |= SCL_PIN /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         	SDA_GPIO->BSRR |= SDA_PIN /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         	SDA_GPIO->BRR  |= SDA_PIN /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      	SCL_GPIO->IDR  & SCL_PIN /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      	SDA_GPIO->IDR  & SDA_PIN /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

#define FALSE 			0
#define TRUE			1

void I2C_GPIO_Config(void);
void I2C_delay(void);
void delay5ms(void);
uint16_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void); 
void I2C_NoAck(void);
uint16_t I2C_WaitAck(void);
void I2C_SendByte(unsigned char SendByte);
unsigned char I2C_RadeByte(void);
uint16_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
uint16_t Multi_Write(unsigned char SlaveAddress,unsigned char REG_Address, uint16_t length, uint8_t* p_REG_data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
uint8_t MultiRead(uint8_t slave_address, uint8_t reg_address, uint16_t data_num, uint8_t* reg_data);

#endif // __MYI2C_H__
