#include "myi2c.h"
#include <string.h>

//#define NEW_I2C

/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  SCL_PIN | SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SCL_GPIO, &GPIO_InitStructure);

}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
		
   uint8_t i = 10; //这里可以优化速度	，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   }  
}

void delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
uint16_t I2C_Start(void)
{
#ifndef NEW_I2C
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return TRUE;
#else	
	SDA_H;
	SCL_H;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_L;
	I2C_delay();
	return TRUE;
#endif
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
#ifndef NEW_I2C
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
#else	
	SDA_L;
	SCL_H;
	I2C_delay();
	SDA_H;
#endif
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{
#ifndef NEW_I2C	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
#else	
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
	SDA_H;
#endif
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
#ifndef NEW_I2C
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
#else
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
#endif
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
uint16_t I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
#ifndef NEW_I2C
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return FALSE;
	}
	SCL_L;
	I2C_delay();
	return TRUE;
#else	
	uint8_t re;
	
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read) {
		re = FALSE;
	} else {
		re = TRUE;
	}
	SCL_L;
	I2C_delay();
	return re;
#endif
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
{
#ifndef NEW_I2C
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
#else	
	for(int i = 0; i<8; i++) {
		if(SendByte&0x80)       SDA_H;  
		else        SDA_L;   
        SendByte<<=1;
		I2C_delay();
		SCL_H;
        I2C_delay();
		SCL_L;
		if(i == 7) SDA_H;
		I2C_delay();
	}
#endif
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;
#ifndef NEW_I2C
    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
#else	
	for(i = 0; i< 8; i++) {
		ReceiveByte<<=1;
		SCL_H;
		I2C_delay();
		if(SDA_read)
		{
			ReceiveByte|=0x01;
		}
		SCL_L;
		I2C_delay();
	}
#endif	
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

uint16_t Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay5ms();
    return TRUE;
}

uint16_t Multi_Write(unsigned char SlaveAddress,unsigned char REG_Address, uint16_t length, uint8_t* p_REG_data)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
	
    I2C_SendByte(REG_Address );   //设置低起始地址      
    if(!I2C_WaitAck()) {
		I2C_Stop();
		return FALSE;
	}
		
	for(int i = 0; i < length; i++) {
		I2C_SendByte(p_REG_data[i]);
		I2C_WaitAck(); 
	}		
    I2C_Stop(); 
    delay5ms();

    return TRUE;
}

//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();return FALSE;}
    I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址      
    if(!I2C_WaitAck()) {
		I2C_Stop();
		return FALSE;
	}
	
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}

//多字节读取
uint8_t MultiRead(uint8_t slave_address, uint8_t reg_address, uint16_t data_num, uint8_t* reg_data)
{
	if(!I2C_Start())
		return FALSE;
	I2C_SendByte(slave_address); 
	if(!I2C_WaitAck()) {
		I2C_Stop();
		return FALSE;
	}
	I2C_SendByte((uint8_t) reg_address);   //设置低起始地址      
	if(!I2C_WaitAck()) {
		I2C_Stop();
		return FALSE;
	}

	I2C_Start();
	I2C_SendByte(slave_address + 1);
	if(!I2C_WaitAck()) {
		I2C_Stop();
		return FALSE;
	}
	
	for(int8_t i = 0; i < data_num - 1; i++) {
		*(reg_data + i) = I2C_RadeByte();
		I2C_Ack();
	}
	
	*(reg_data + data_num - 1) = I2C_RadeByte();
	I2C_NoAck();
    I2C_Stop();
	
	return TRUE;
}

int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata)
{
	*pdata = Single_Read(address, index);
	return 0;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
	if(MultiRead(address, index, (uint16_t)count, pdata) == TRUE) 
		return 0;
	else
		return -1;
}

int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata)
{
	uint8_t data[2];
	if(MultiRead(address, index, 2, (uint8_t*)data) != TRUE)
		return -1;
	*pdata = ((uint16_t)data[0]<<8) + ((uint16_t)data[1]);
	return 0;

}

int32_t VL53L0X_read_dword(uint8_t address,  uint8_t index, uint32_t *pdata)
{
	uint8_t data[4];
	if(MultiRead(address, index, 4, (uint8_t*)data) != TRUE)
		return -1;
	*pdata = ((uint16_t)data[0]<<24) + ((uint16_t)data[1]<<16) + ((uint16_t)data[2]<<8) + (uint16_t)data[3];
	return 0;
}

int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t  data)
{
	Single_Write(address, index, data);
	return 0;
}

int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data)
{
	uint8_t send_buf[2] = {0};
	send_buf[0] = (uint8_t)(data>>8);
	send_buf[1] = (uint8_t)(data&0x00FF);

	if(Multi_Write(address, index, 2, send_buf) == TRUE) 
		return 0;
	else 
		return -1;
}

int32_t VL53L0X_write_dword(uint8_t address,  uint8_t index, uint32_t  data)
{
	uint8_t send_buf[4] = {0};
	send_buf[0] = (uint8_t)(data>>24);
	send_buf[1] = (uint8_t)((data&0x00FF0000) >> 16);
	send_buf[2] = (uint8_t)((data&0x0000FF00) >> 8);
	send_buf[3] = (uint8_t)(data&0x0000FF);
	if(Multi_Write(address, index, 4, send_buf) == TRUE) 
		return 0;
	else 
		return -1;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
	if(Multi_Write(address, index, count, pdata) == TRUE) 
		return 0;
	else 
		return -1;
}
