#include "I2C.h"
#include <stdbool.h>

/*******************************************************************************
* Function Name  : I2C_DevInit
* Description    : Initializes peripherals used by the I2C EEPROM driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DevInit(void)
{	
   	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/** I2C1 GPIO Configuration			 
	PB8	 ------> I2C1_SCL		 
	PB9	 ------> I2C1_SDA	*/	
	/*Enable or disable the AHB peripheral clock */	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	/*Configure GPIO pin */
	
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_pin | I2C_SDA_pin;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(I2C_GPIO, &GPIO_InitStructure);
	
	/*Configure GPIO pin alternate function */	
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource9, GPIO_AF_4);	
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource10, GPIO_AF_4);

	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;	
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_DigitalFilter = 0x01;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	//I2C_InitStructure.I2C_Timing = 0x10400BDB;//100Khz
	I2C_InitStructure.I2C_Timing =0x0090174F;//400Khz
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	
	I2C_Init(I2C1, &I2C_InitStructure);	
	I2C_Cmd(I2C1, ENABLE);
	return;

}

/*******************************************************************************
* Function Name  : I2C_Write
* Description    : Writes more than one byte to the Device with a single WRITE
*                  cycle. The number of byte can't exceed the Device page size.
* Input          : - pBuffer : pointer to the buffer containing the data to be 
*                    written to the Device.
*                  - WriteAddr : Device's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the Device.
* Output         : None
* Return         : true of false
*******************************************************************************/
bool  I2C_Write(uint8_t DevAddr, uint8_t RegAddr, uint8_t* pBuffer, uint8_t NumByteToWrite)
{
	uint16_t timeout = I2C_TIMEOUT;
	
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}
	
	/* Send Touch address for write */	
	I2C_TransferHandling(I2C1, DevAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	
	timeout = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS)==RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}
	
	I2C_SendData(I2C1, RegAddr);

	timeout = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TCR) == RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}
	
	I2C_TransferHandling(I2C1, DevAddr, NumByteToWrite, I2C_AutoEnd_Mode, I2C_No_StartStop);
	
	/* While there is data to be written */
	while(NumByteToWrite--)  
	{
		timeout = I2C_TIMEOUT;
		/* Test on EV8 and clear it */
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET)
		{
			if((timeout--) == 0)
			{
				return false;
			}
		}
		/* Send the current byte */
		I2C_SendData(I2C1, *pBuffer); 

		/* Point to the next byte to be written */
		pBuffer++; 		
	}
	/* Send STOP condition */
	timeout = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}
	
	return true;
	
}

/*******************************************************************************
* Function Name  : I2C_Read
* Description    : Reads a block of data from the Device.
* Input          : - pBuffer : pointer to the buffer that receives the data read 
*                    from the Device.
*                  - ReadAddr : Dev's internal register address to read from.
*                  - NumByteToRead : number of bytes to read from the Device.
* Output         : None
* Return         : true or false 
*******************************************************************************/
bool  I2C_Read(uint8_t DevAddr, uint8_t RegAddr, uint8_t* pBuffer, uint8_t NumByteToRead)
{  
	uint16_t timeout = I2C_TIMEOUT;

	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}

	/* Generate start & wait event detection */
	I2C_TransferHandling(I2C1, DevAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);


	timeout = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}

	I2C_SendData(I2C1, RegAddr);

	timeout = I2C_TIMEOUT;	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}

	/* Send STRAT condition a second time */  
	I2C_TransferHandling(I2C1, DevAddr, NumByteToRead,  I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	
	/* While there is data to be read */
	while(NumByteToRead)  
	{	
		timeout = I2C_TIMEOUT;
		
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
		{
			if((timeout--) == 0)
			{
				return false;
			}
		}

		/* Read a byte from the EEPROM */
		*pBuffer = I2C_ReceiveData(I2C1);

		/* Point to the next location where the byte read will be s**ed */
		pBuffer++; 

		/* Decrement the read bytes counter */
		NumByteToRead--;        
	}
	
	/* Enable Acknowledgement to be ready for another reception */
	timeout = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}
	return true;
}

