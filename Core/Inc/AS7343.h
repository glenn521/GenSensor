/*
 * AS7343.h
 *
 *  Created on: Jan 4, 2023
 *      Author: Yousif Al-Khoury
 */

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

#ifndef INC_AS7343_H_
#define INC_AS7343_H_

// Slave Address
#define AS7343_ADDR_A 		0x39
#define AS7343_ADDR_B 		0x48

// Register Address
#define AS7343_AUXID 		0x58
#define AS7343_REVID 		0x59
#define AS7343_ID 			0x5A
#define AS7343_CFG12			0x66
#define AS7343_ENABLE		0x80
#define AS7343_ATIME			0x81
#define AS7343_WTIME			0x83
#define AS7343_SP_TH_L_LSB	0x84
#define AS7343_SP_TH_L_MSB	0x85
#define AS7343_SP_TH_H_LSB	0x86
#define AS7343_SP_TH_H_MSB	0x87
#define AS7343_STATUS		0x93
#define AS7343_ASTATUS		0x94

// Spectrum Data Registers
#define AS7343_DATA_0_L		0x95
#define AS7343_DATA_0_H		0x96

#define AS7343_DATA_1_L		0x97
#define AS7343_DATA_1_H		0x98

#define AS7343_DATA_2_L		0x99
#define AS7343_DATA_2_H		0x9A

#define AS7343_DATA_3_L		0x9B
#define AS7343_DATA_3_H		0x9C

#define AS7343_DATA_4_L		0x9D
#define AS7343_DATA_4_H		0x9E

#define AS7343_DATA_5_L		0x9F
#define AS7343_DATA_5_H		0xA0

#define AS7343_DATA_6_L		0xA1
#define AS7343_DATA_6_H		0xA2

#define AS7343_DATA_7_L		0xA3
#define AS7343_DATA_7_H		0xA4

#define AS7343_DATA_8_L		0xA5
#define AS7343_DATA_8_H		0xA6

#define AS7343_DATA_9_L		0xA7
#define AS7343_DATA_9_H		0xA8

#define AS7343_DATA_10_L		0xA9
#define AS7343_DATA_10_H		0xAA

#define AS7343_DATA_11_L		0xAB
#define AS7343_DATA_11_H		0xAC

#define AS7343_DATA_12_L		0xAD
#define AS7343_DATA_12_H		0xAE

#define AS7343_DATA_13_L		0xAF
#define AS7343_DATA_13_H		0xB0

#define AS7343_DATA_14_L		0xB1
#define AS7343_DATA_14_H		0xB2

#define AS7343_DATA_15_L		0xB3
#define AS7343_DATA_15_H		0xB4

#define AS7343_DATA_16_L		0xB5
#define AS7343_DATA_16_H		0xB6

#define AS7343_DATA_17_L		0xB7
#define AS7343_DATA_17_H		0xB8

#define AS7343_STATUS2		0x90
#define AS7343_STATUS3		0x91
#define AS7343_STATUS5		0xBB
#define AS7343_STATUS4		0xBC
#define AS7343_CFG0			0xBF
#define AS7343_CFG1			0xC6
#define AS7343_CFG3			0xC7
#define AS7343_CFG6			0xF5
#define AS7343_CFG8			0xC9
#define AS7343_CFG9			0xCA
#define AS7343_CFG10			0x65
#define AS7343_PERS			0xCF
#define AS7343_GPIO			0x6B
#define AS7343_ASTEP_LSB		0xD4
#define AS7343_ASTEP_MSB		0xD5
#define AS7343_CFG20			0xD6
#define AS7343_LED			0xCD
#define AS7343_AGC_GAIN_MAX	0xD7
#define AS7343_AZ_CONFIG		0xDE
#define AS7343_FD_TIME_1		0xE0
#define AS7343_FD_TIME_2		0xE2
#define AS7343_FD_CFG0		0xDF
#define AS7343_FD_STATUS		0xE3
#define AS7343_INTENAB		0xF9
#define AS7343_CONTROL		0xFA
#define AS7343_FIFO_MAP		0xFC
#define AS7343_FIFO_LVL		0xFD
#define AS7343_FDATA_L		0xFE
#define AS7343_FDATA_H		0xFF

typedef struct {

	/**
	 * The handle to the I2C bus for the device.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The I2C device address.
	 */
	uint16_t device_address;

} as7343_handle_t;

uint8_t AS7343_read(as7343_handle_t *handle, uint8_t regAddress);
bool AS7343_write(as7343_handle_t *handle, uint8_t regAddress, uint8_t data);
void AS7343_power(as7343_handle_t *handle, bool power);
uint16_t AS7343_read_spectral_data(as7343_handle_t *handle, uint8_t channel);
void AS7343_enable_spectral_measurement(as7343_handle_t *handle, bool enable);
void AS7343_set_cycle(as7343_handle_t *handle, uint8_t cycle);
void AS7343_set_SMUX_F1_F4(as7343_handle_t *handle);
void AS7343_set_SMUX_F5_F8(as7343_handle_t *handle);
void AS7343_direct_config_3_chain(as7343_handle_t *handle);
bool AS7343_get_AVALID_bit(as7343_handle_t *handle);
bool AS7343_check_saturation(as7343_handle_t *handle);
bool AS7343_check_analog_saturation(as7343_handle_t *handle);
void AS7343_delay_for_reading(as7343_handle_t *handle);
void AS7343_read_18(as7343_handle_t *handle, uint16_t* data);
void AS7343_set_gain(as7343_handle_t *handle, uint8_t gain_raw);
uint8_t AS7343_get_gain(as7343_handle_t *handle);
void AS7343_set_TINT(as7343_handle_t *handle, double TINT);
double AS7343_get_TINT(as7343_handle_t *handle);
void AS7343_optimizer(as7343_handle_t *handle, float max_TINT);


#endif /* INC_AS7343_H_ */
