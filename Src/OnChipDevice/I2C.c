#include "I2C.h"

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
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	/*Configure GPIO pin */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*Configure GPIO pin alternate function */	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);

	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;	
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_DigitalFilter = 0x01;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	//I2C_InitStructure.I2C_Timing = 0x40B22536;//100Khz
	I2C_InitStructure.I2C_Timing =0x10950C27;//400Khz
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
* Return         : None
*******************************************************************************/
BOOLEAN  I2C_Write(INT8U DevAddr, INT8U RegAddr,INT8U* pBuffer, INT8U NumByteToWrite)
{
	INT16U timeout = I2C_TIMEOUT;
	
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}
	
	/* Send Touch address for write */	
	I2C_TransferHandling(I2C1, (DevAddr<<1), 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	
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
	
	I2C_TransferHandling(I2C1, (DevAddr<<1), NumByteToWrite, I2C_AutoEnd_Mode, I2C_No_StartStop);
	
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
*                  - ReadAddr : EEPROM's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the Device.
* Output         : None
* Return         : None
*******************************************************************************/
BOOLEAN  I2C_Read(INT8U DevAddr, INT8U RegAddr, INT8U* pBuffer, INT8U NumByteToRead)
{  
	INT16U timeout = 60;

	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) != RESET)
	{
		if((timeout--) == 0)
		{
			return false;
		}
	}

	/* Generate start & wait event detection */
	I2C_TransferHandling(I2C1, (DevAddr<<1), 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);


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
	I2C_TransferHandling(I2C1, (DevAddr<<1), NumByteToRead,  I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	
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