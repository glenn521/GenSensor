/*
 *  PCA9548a.c
 *
 *  PCA 9548a I2C multiplexer stm32 driver
 *
 *  Created on: Jan 4, 2023
 *      Author: Yousif Al-Khoury
 */

#include "main.h"
#include "PCA9548a.h"

void PCA9548a_Select(pca9548a_handle_t *handle, uint8_t channel) {
	//Select Mux channel
	uint8_t data = 1 << channel;

	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address << 1, &data, 1, 10);
}

void PCA9548a_Disable(pca9548a_handle_t *handle) {
	//Disable currently selected Mux channel
	uint8_t data = 0;

	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address << 1, &data, 1, 10);
}
