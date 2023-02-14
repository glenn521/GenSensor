/*
 * PASCO2.c
 *
 *  Created on: Jan 4, 2023
 *      Author: Yousif Al-Khoury
 */

#include "main.h"
#include "PASCO2.h"

#include <assert.h>

//bool PASCO2_init(pasco2_handle_t *handle){
//	assert(handle->i2c_handle->Init.NoStretchMode == I2C_NOSTRETCH_DISABLE);
//
//
//}

uint8_t PASCO2_read(pasco2_handle_t *handle, uint8_t regAddress){
	uint8_t rx_buffer;

	HAL_I2C_Mem_Read(handle->i2c_handle, handle->device_address<<1, regAddress, 1, &rx_buffer, 1, 10);

	return rx_buffer;
}

bool PASCO2_write(pasco2_handle_t *handle, uint8_t regAddress, uint8_t data){
	if(HAL_I2C_Mem_Write(handle->i2c_handle, handle->device_address<<1, regAddress, 1, &data, 1, 10) == HAL_OK) {
		return true;
	}
	return false;
}

void PASCO2_init(pasco2_handle_t *handle) {
	//set pressure
	PASCO2_write(handle, PASCO2_PRESS_REF_H, 0x03);
	PASCO2_write(handle, PASCO2_PRESS_REF_L, 0xF5);
}

bool PASCO2_get_status(pasco2_handle_t *handle) {
	uint8_t pasco2_status = PASCO2_read(handle, PASCO2_SENS_STS);

	if(pasco2_status == 0xC0) {
		return true;
	}
	else {
		return false;
	}
}

uint16_t PASCO2_get_ppm(pasco2_handle_t *handle) {

	PASCO2_write(handle, PASCO2_MEAS_CFG, 0x01);

	HAL_Delay(1000);
	uint8_t ppm_h = PASCO2_read(handle, PASCO2_CO2PPM_H);
	HAL_Delay(10);
	uint8_t ppm_l = PASCO2_read(handle, PASCO2_CO2PPM_L);

	uint16_t ppm = (((uint16_t)ppm_h)<<8) | ppm_l;

	return ppm;
}
