#ifndef __VL53L0X_H__
#define __VL53L0X_H__

#include "stm32f0xx.h"
#include "vl53l0x_api.h"

extern VL53L0X_Dev_t		MyDevice;


void VL53L0XInit(void);

int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata);
int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata);
int32_t VL53L0X_read_dword(uint8_t address,  uint8_t index, uint32_t *pdata);
int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count);
int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data);
int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data);
int32_t VL53L0X_write_dword(uint8_t address,  uint8_t index, uint32_t  data);
int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count);

#endif
