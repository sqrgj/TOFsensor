#include "vl53l0x_api.h"
#include "myi2c.h"
#include "vl53l0x_driver.h"

VL53L0X_Error Status = VL53L0X_ERROR_NONE;
FixPoint1616_t LimitCheckCurrent;
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;
FixPoint1616_t CalDistanceMilliMeter = 100;
int32_t OffsetMicroMeter;
FixPoint1616_t XTalkCalDistance = 100;
FixPoint1616_t XTalkCompensationRateMegaCps;

VL53L0X_Dev_t 					MyDevice;
VL53L0X_Version_t               Version;
VL53L0X_DeviceInfo_t            DeviceInfo;

void VL53L0XInit(void)
{
	// Initialize Comms
	

	/*
     *  Get the version of the VL53L0X API running in the firmware
     */
		MyDevice.I2cDevAddr      = 	0x52;
		MyDevice.comms_type      =  1;
		MyDevice.comms_speed_khz =  400;
		
//		Status = VL53L0X_SetGpioConfig(&MyDevice[i], 0, VL53L0X_DEVICEMODE_SINGLE_RANGING,VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_LOW,
//										VL53L0X_INTERRUPTPOLARITY_LOW);//VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY
			
			//Status=VL53L0X_SetDeviceAddress(&MyDevice[i],i+10);
			//MyDevice[i].I2cDevAddr      = (i+10)<<1;
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
		
		
		if(Status == VL53L0X_ERROR_NONE)
		{
			Status = VL53L0X_SetDeviceMode(&MyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode;
		}
		Status =	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice, 20000);
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
			//Status =  VL53L0X_StartMeasurement(&MyDevice[i]);;
//		Status = VL53L0X_SetLimitCheckEnable(&MyDevice[i],
//												VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
//		Status = VL53L0X_SetLimitCheckValue(&MyDevice[i],
//											VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.40*65536);
		


		VL53L0X_StartMeasurement(&MyDevice);
	 
	
}

