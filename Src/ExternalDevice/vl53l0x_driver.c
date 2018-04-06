#include "vl53l0x_api.h"
#include "vl53l0x_driver.h"
#include "I2C.h"


VL53L0X_Dev_t 						MyDevice;
VL53L0X_RangingMeasurementData_t    RangingMeasurementData;


void VL53L0XInit(void)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Version_t               Version;
	VL53L0X_DeviceInfo_t            DeviceInfo;
	//variables for calibration
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
//	FixPoint1616_t CalDistanceMilliMeter = 100;
//	int32_t OffsetMicroMeter;
//	FixPoint1616_t XTalkCalDistance = 100;
//	FixPoint1616_t XTalkCompensationRateMegaCps;
	
	
	// Initialize Comms
	MyDevice.I2cDevAddr      = 	0x52;
	MyDevice.comms_type      =  1;
	MyDevice.comms_speed_khz =  400;
		
//		Status = VL53L0X_SetGpioConfig(&MyDevice[i], 0, VL53L0X_DEVICEMODE_SINGLE_RANGING,VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_LOW,
//										VL53L0X_INTERRUPTPOLARITY_LOW);//VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY
	/*
	 *  Get the version of the VL53L0X API running in the firmware
	 */
	if(Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_GetVersion(&Version);
		if (Status != 0)
			Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	}
		
	if(Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_DataInit(&MyDevice); // Data initialization
	}
	
	if(Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
	}
	
	if(Status == VL53L0X_ERROR_NONE)
	{
		Status = VL53L0X_StaticInit(&MyDevice);
	}
	
	////////////////////////Calibration//////////////////////////
		
	if(Status == VL53L0X_ERROR_NONE)
	{
		Status =  VL53L0X_PerformRefSpadManagement(&MyDevice, &refSpadCount, &isApertureSpads);
	}
	
	if(Status == VL53L0X_ERROR_NONE)
	{
		Status =  VL53L0X_PerformRefCalibration(&MyDevice, &VhvSettings, &PhaseCal);
	}
		
//	if(Status == VL53L0X_ERROR_NONE)
//    {
//		Status =  VL53L0X_PerformOffsetCalibration(&MyDevice[i], CalDistanceMilliMeter, &OffsetMicroMeter);
//    }
//	
//	
//	if(Status == VL53L0X_ERROR_NONE)
//    {
//		Status =  VL53L0X_PerformXTalkCalibration(&MyDevice[i], XTalkCalDistance, &XTalkCompensationRateMegaCps);
//    }

	//////////////////////////Calibration end////////////////////
	
	
	if(Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetDeviceMode(&MyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode;
	}
	if(Status == VL53L0X_ERROR_NONE) {
		Status =	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice, 20000);
	}
	
	// Enable/Disable Sigma and Signal check
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(&MyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(&MyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(&MyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(&MyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
	}
//		Status = VL53L0X_SetLimitCheckEnable(&MyDevice[i],
//												VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
//		Status = VL53L0X_SetLimitCheckValue(&MyDevice[i],
//											VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.40*65536);
	
	VL53L0X_StartMeasurement(&MyDevice);

}

void VL53L0X_ReadTask(void)
{
	VL53L0X_GetRangingMeasurementData(&MyDevice, &RangingMeasurementData);
}

int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata)
{
	if(I2C_Read(address, index, pdata, 1))
		return 0;
	else
		return -1;
}

int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata)
{
	uint8_t data[2];
	
	if(I2C_Read(address, index, data, 2)) {
		*pdata = ((uint16_t)data[0]<<8) + ((uint16_t)data[1]);
		return 0;
	}
	else
		return -1;

}

int32_t VL53L0X_read_dword(uint8_t address,  uint8_t index, uint32_t *pdata)
{
	uint8_t data[4];
	
	if(I2C_Read(address, index, data, 4)) {
		*pdata = ((uint32_t)data[0]<<24) + ((uint32_t)data[1]<<16) + ((uint32_t)data[2]<<8) + (uint32_t)data[3];
		return 0;
	}
	else
		return -1;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
	if(I2C_Read(address, index, pdata, (uint8_t)count)) {
		return 0;
	}
	else
		return -1;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
	if(I2C_Write(address, index, &data, 1)) {
		return 0;
	}
	else
		return -1;
}

int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data)
{
	uint8_t send_buf[2] = {0};
	send_buf[0] = (uint8_t)(data>>8);
	send_buf[1] = (uint8_t)(data&0x00FF);

	if(I2C_Write(address, index, send_buf, 2)) {
		return 0;
	}
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
	
	if(I2C_Write(address, index, send_buf, 4)) {
		return 0;
	}
	else
		return -1;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
	if(I2C_Write(address, index, pdata, (uint8_t)count)) {
		return 0;
	}
	else
		return -1;
}



