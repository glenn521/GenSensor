/*
 * VEML6031.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Yousif
 */


#include "main.h"
#include "VEML6031.h"

#include <assert.h>

uint8_t VEML6031_read(veml6031_handle_t* handle, uint8_t reg_address) {
	uint8_t rx_buffer;

	HAL_I2C_Mem_Read(handle->i2c_handle, handle->device_address<<1, reg_address, 1, &rx_buffer, 1, 10);

	return rx_buffer;
}

bool VEML6031_write(veml6031_handle_t* handle, uint8_t reg_address, uint8_t data) {
	if(HAL_I2C_Mem_Write(handle->i2c_handle, handle->device_address<<1, reg_address, 1, &data, 1, 10) == HAL_OK) {
		return true;
	}
	return false;
}

float VEML6031_optimizer(veml6031_handle_t* handle) {
	float resolution = 0.0034;  // resolution with TINT = 400ms and gain = 2
	uint8_t current_TINT = 0x07;
	uint8_t current_GAIN = 0x19;
	bool max_saturation = false;
	VEML6031_write(handle, ALS_CONF_0, 0x71);
	VEML6031_write(handle, ALS_CONF_1, 0x88);

	uint8_t conf0 = VEML6031_read(handle, ALS_CONF_0) & 0x8f;
	uint8_t conf1 = VEML6031_read(handle, ALS_CONF_1) & 0xe7;

	while(true){
		uint16_t data = VEML6031_read_light(handle);

		if(data>0.8*0xFFFF) { //change resolution if data is close to saturation
			//change gain
			if(current_TINT == 0x1) {
				if(current_GAIN == 0x3) { //min gain and TINT reached
					//change PD
					if(max_saturation) {
						return 0;
					}
					max_saturation = true;
					current_GAIN = 0x32;
					VEML6031_write(handle, ALS_CONF_1, conf1 | 0x4);
				}
				else {
					current_GAIN = current_GAIN>>1;
					VEML6031_write(handle, ALS_CONF_1, conf1 | ((current_GAIN & 0x03)<<3));
				}
			}
			//change integration time
			else {
				current_TINT -= 0x1; //change TINT
				VEML6031_write(handle, ALS_CONF_0, conf0 | current_TINT << 4);
			}
			resolution *= 2.0;
		}
		else {
			break;
		}
	}

	return resolution;
}

uint16_t VEML6031_read_light(veml6031_handle_t* handle) {
//	VEML6031_write(handle, ALS_CONF_0, 0x71);
//	VEML6031_write(handle, ALS_CONF_1, 0x88);
	uint8_t conf0 = VEML6031_read(handle, ALS_CONF_0);
	uint8_t conf1 = VEML6031_read(handle, ALS_CONF_1);

	//Trigger reading
	VEML6031_write(handle, ALS_CONF_0, conf0 & 0xFE);
	VEML6031_write(handle, ALS_CONF_1, conf1^0x81);

//	VEML6031_write(handle, ALS_CONF_0, conf0 | 0x04);
	HAL_Delay(500);


	uint8_t data_L = VEML6031_read(handle, ALS_DATA_L);
	uint8_t data_H = VEML6031_read(handle, ALS_DATA_H);

	uint16_t light_intensity = (((uint16_t)data_H)<<8) | data_L;

	//Sleep
//	VEML6031_write(handle, ALS_CONF_0, conf0 | 0x01);
//	VEML6031_write(handle, ALS_CONF_1, conf1^0x81);
//	HAL_Delay(100);

	return light_intensity;
}
