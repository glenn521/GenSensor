/*
 *  PCA9548a.c
 *
 *  PCA 9548a I2C multiplexer stm32 driver
 *
 *  Created on: Jan 4, 2023
 *      Author: Yousif Al-Khoury
 */
#include "stm32f4xx_hal.h"

#ifndef INC_PCA9548A_H_
#define INC_PCA9548A_H_

#define PCA9548_ADDR0 0x70
#define PCA9548_ADDR1 0x71
#define PCA9548_ADDR2 0x72
#define PCA9548_ADDR3 0x73
#define PCA9548_ADDR4 0x74
#define PCA9548_ADDR5 0x75
#define PCA9548_ADDR6 0x76
#define PCA9548_ADDR7 0x77

typedef struct {

	/**
	 * The handle to the I2C bus for the device.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The I2C device address.
	 */
	uint16_t device_address;

} pca9548a_handle_t;

void PCA9548a_Select(pca9548a_handle_t *handle, uint8_t channel);
void PCA9548a_Disable(pca9548a_handle_t *handle);

#endif /* INC_PCA9548A_H_ */
