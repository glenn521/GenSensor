/*
 * VEML6031.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Yousif
 */

#ifndef INC_VEML6031_H_
#define INC_VEML6031_H_

#include <stdbool.h>

#define VEML6031_I2C_ADDRESS 0x29

#define ALS_CONF_0 		0x00
#define ALS_CONF_1 		0x01
#define ALS_WH_L 		0x04
#define ALS_WH_H 		0x05
#define ALS_WL_L 		0x06
#define ALS_WL_H 		0x07
#define ALS_DATA_L 		0x10
#define ALS_DATA_H 		0x11
#define IR_DATA_L 		0x12
#define IR_DATA_H 		0x13
#define ID_L 			0x14
#define ID_H 			0x15
#define ALS_INT 		0x17

typedef struct {

	/**
	 * The handle to the I2C bus for the device.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The I2C device address.
	 */
	uint16_t device_address;

} veml6031_handle_t;

uint8_t VEML6031_read(veml6031_handle_t* handle, uint8_t reg_address);
bool VEML6031_write(veml6031_handle_t* handle, uint8_t reg_address, uint8_t data);
float VEML6031_optimizer(veml6031_handle_t* handle);
uint16_t VEML6031_read_light(veml6031_handle_t* handle);

#endif /* INC_VEML6031_H_ */
