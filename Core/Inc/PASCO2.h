/*
 * PASCO2.h
 *
 *  Created on: Jan 4, 2023
 *      Author: Yousif Al-Khoury
 */
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#ifndef INC_PASCO2_H_
#define INC_PASCO2_H_

// I2C Slave Address
#define PASCO2_ADDR 		0x28

#define PASCO2_PROD_ID		0x00
#define PASCO2_SENS_STS		0x01
#define PASCO2_MEAS_RATE_H	0x02
#define PASCO2_MEAS_RATE_L	0x03
#define PASCO2_MEAS_CFG		0x04
#define PASCO2_CO2PPM_H		0x05
#define PASCO2_CO2PPM_L		0x06
#define PASCO2_MEAS_STS		0x07
#define PASCO2_INT_CFG		0x08
#define PASCO2_ALARM_TH_H	0x09
#define PASCO2_ALARM_TH_L	0x0A
#define PASCO2_PRESS_REF_H	0x0B
#define PASCO2_PRESS_REF_L	0x0C
#define PASCO2_CALIB_REF_H	0x0D
#define PASCO2_CALIB_REF_L	0x0E
#define PASCO2_SCRATCH_PAD	0x0F
#define PASCO2_SENS_RST		0x10

typedef struct {

	/**
	 * The handle to the I2C bus for the device.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The I2C device address.
	 */
	uint16_t device_address;

} pasco2_handle_t;

uint8_t PASCO2_read(pasco2_handle_t *handle, uint8_t regAddress);
bool PASCO2_write(pasco2_handle_t *handle, uint8_t regAddress, uint8_t data);
void PASCO2_init(pasco2_handle_t *handle);
bool PASCO2_get_status(pasco2_handle_t *handle);
uint16_t PASCO2_get_ppm(pasco2_handle_t *handle);
#endif /* INC_PASCO2_H_ */
