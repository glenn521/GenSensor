/*
 * AS7343.c
 *
 *  Created on: Jan 11, 2023
 *      Author: Yousif
 */

#include "main.h"
#include "AS7343.h"

#include <assert.h>

uint8_t AS7343_read(as7343_handle_t *handle, uint8_t regAddress){
	uint8_t rx_buffer;
	HAL_I2C_Mem_Read(handle->i2c_handle, handle->device_address<<1, regAddress, 1, &rx_buffer, 1, 10);

	return rx_buffer;
}

bool AS7343_write(as7343_handle_t *handle, uint8_t regAddress, uint8_t data){
	if(HAL_I2C_Mem_Write(handle->i2c_handle, handle->device_address<<1, regAddress, 1, &data, 1, 10) == HAL_OK) {
		return true;
	}
	return false;
}

void AS7343_power(as7343_handle_t *handle, bool power){

	AS7343_write(handle, AS7343_ENABLE, 0x01);
}

uint16_t AS7343_read_spectral_data(as7343_handle_t *handle, uint8_t channel) {
	uint8_t rx_buffer[2];
	HAL_I2C_Mem_Read(handle->i2c_handle, handle->device_address<<1, AS7343_DATA_0_L + (2*channel), 1, rx_buffer, 2, 10);

	return ((uint16_t)rx_buffer[1]<<8) | (uint16_t)rx_buffer[0];
}

void AS7343_enable_spectral_measurement(as7343_handle_t *handle, bool enable) {
	AS7343_write(handle, AS7343_ENABLE, 0x01 | ((uint8_t)enable)<<1);
}

void AS7343_set_cycle(as7343_handle_t *handle, uint8_t cycle) {
	AS7343_write(handle, AS7343_CFG20, cycle<<5);
}

void AS7343_set_SMUX_F1_F4(as7343_handle_t *handle){
	AS7343_write(handle, AS7343_CFG9, 0x10);
	AS7343_write(handle, AS7343_INTENAB, 0x01);
	AS7343_write(handle, AS7343_CFG6, 0x10);
	AS7343_write(handle, 0x00, 0x05);
	AS7343_write(handle, 0x01, 0x02);
	AS7343_write(handle, 0x02, 0x10);
	AS7343_write(handle, 0x03, 0x04);
	AS7343_write(handle, 0x04, 0x55);
	AS7343_write(handle, 0x05, 0x00);
	AS7343_write(handle, 0x06, 0x30);
	AS7343_write(handle, 0x07, 0x05);
	AS7343_write(handle, 0x08, 0x06);
	AS7343_write(handle, 0x09, 0x00);
	AS7343_write(handle, 0x80, 0x11);
}

void AS7343_set_SMUX_F5_F8(as7343_handle_t *handle){
	AS7343_write(handle, AS7343_CFG9, 0x10);
	AS7343_write(handle, AS7343_INTENAB, 0x01);
	AS7343_write(handle, AS7343_CFG6, 0x10);
	AS7343_write(handle, 0x00, 0x05);
	AS7343_write(handle, 0x01, 0x20);
	AS7343_write(handle, 0x02, 0x00);
	AS7343_write(handle, 0x03, 0x40);
	AS7343_write(handle, 0x04, 0x55);
	AS7343_write(handle, 0x05, 0x31);
	AS7343_write(handle, 0x06, 0x00);
	AS7343_write(handle, 0x07, 0x05);
	AS7343_write(handle, 0x08, 0x06);
	AS7343_write(handle, 0x09, 0x00);
	AS7343_write(handle, 0x80, 0x11);
}

void AS7343_direct_config_3_chain(as7343_handle_t *handle) {
	AS7343_write(handle, AS7343_CFG6, 0x0);
	AS7343_write(handle, AS7343_FD_CFG0, 0xa1);
	AS7343_write(handle, AS7343_CFG10, 0xf2);

	AS7343_write(handle, AS7343_CFG0, 0x10);
	AS7343_write(handle, AS7343_CFG1, 0x0c);
	AS7343_write(handle, AS7343_CFG8, 0xc8);
	AS7343_write(handle, AS7343_CFG20, 0x62);
	AS7343_write(handle, AS7343_AGC_GAIN_MAX, 0x99);
	AS7343_write(handle, AS7343_FD_TIME_1, 0x64);
	AS7343_write(handle, AS7343_FD_TIME_2, 0x21);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x04);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x65);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x02);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x05);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x01);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x30);
	AS7343_write(handle, 0xe4, 0x46);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x60);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x20);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x04);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x50);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x03);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x01);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x05);
	AS7343_write(handle, 0xe4, 0x56);

	AS7343_write(handle, 0xe7, 0x05);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x60);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x30);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x40);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x10);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x20);
	AS7343_write(handle, 0xe4, 0x66);

	AS7343_write(handle, 0xe7, 0x00);
	AS7343_write(handle, 0xe4, 0x66);

//	AS7343_write(handle, 0x80, 0x11);
}

bool AS7343_get_AVALID_bit(as7343_handle_t *handle) {
	uint8_t status2 = AS7343_read(handle, AS7343_STATUS2);

	return (status2 & 0x40) > 0;
}

bool AS7343_check_saturation(as7343_handle_t *handle) {
	uint8_t status2 = AS7343_read(handle, AS7343_STATUS2);

	return (status2 & 0x10) > 0;
}

bool AS7343_check_analog_saturation(as7343_handle_t *handle) {
	uint8_t status2 = AS7343_read(handle, AS7343_STATUS2);

	return (status2 & 0x10) > 0;
}

void AS7343_delay_for_reading(as7343_handle_t *handle) {
	while(!AS7343_get_AVALID_bit(handle)) {
		HAL_Delay(10);
	}
}

void AS7343_read_18(as7343_handle_t *handle, uint16_t* data) {
	AS7343_enable_spectral_measurement(handle, true);
	AS7343_delay_for_reading(handle);
	AS7343_enable_spectral_measurement(handle, false);

	for(uint8_t i = 0; i<18; i++) {
		data[i] = AS7343_read_spectral_data(handle, i);
	}

}

void AS7343_set_gain(as7343_handle_t *handle, uint8_t gain_raw) {
	AS7343_write(handle, AS7343_CFG1, gain_raw);
}

uint8_t AS7343_get_gain(as7343_handle_t *handle) {
	return AS7343_read(handle, AS7343_CFG1);
}

void AS7343_set_TINT(as7343_handle_t *handle, double TINT) {
	uint8_t ATIME = 0x00;
	uint16_t ASTEP = 0x0000;
	while(true) {
		ASTEP = ((TINT/(double)(ATIME+1))*720.0/2.0);

		if(abs(((ATIME+1)*(ASTEP+1)*2/720) - (uint16_t)TINT) <=1) {
			break;
		}
		else {
			ATIME += 1;
		}
	}
	AS7343_write(handle, AS7343_ATIME, ATIME);
	AS7343_write(handle, AS7343_ASTEP_LSB, (uint8_t)(ASTEP & 0xFF));
	AS7343_write(handle, AS7343_ASTEP_MSB, (uint8_t)(ASTEP>>8));
}

double AS7343_get_TINT(as7343_handle_t *handle) {
	uint16_t ASTEP = (AS7343_read(handle, AS7343_ASTEP_MSB) << 8) | (AS7343_read(handle, AS7343_ASTEP_LSB));
	uint8_t ATIME = AS7343_read(handle, AS7343_ATIME);

	double TINT = (ASTEP+1)*(ATIME+1)*(2.0/720.0);

	return TINT;
}

void AS7343_optimizer(as7343_handle_t *handle, float max_TINT) {
	uint8_t currentGain = 12;

	uint16_t FSR = 65535;
	float TINT = 182.0;
	AS7343_set_TINT(handle, TINT);

	uint16_t max_count;
	uint16_t min_count;

	while(true) {
		max_count = 0;
		min_count = 0xffff;
		AS7343_set_gain(handle, currentGain);

		uint16_t data[18];
		AS7343_enable_spectral_measurement(handle, true);
		AS7343_read_18(handle, data);
		AS7343_enable_spectral_measurement(handle, false);

		for(uint8_t i = 0; i<18; i++){
			if(i == 5 || i == 11 || i == 17) {
				continue;
			}
			if(data[i]>max_count) {
				max_count = data[i];
			}
			if(data[i]<min_count) {
				min_count = data[i];
			}
		}

		if (max_count > 0xE665) {
			if(currentGain == 0){
				//TODO: send optimizer failed due to saturation message
				break;
			}
			currentGain -= 1;
			continue;
		}

		else if (min_count == 0) {
			if(currentGain == 12){
				//TODO: send optimizer failed due to saturation message
				break;
			}
			currentGain += 1;
			continue;
		}

		else {

			break;
		}
	}

	float counts_expected = (float) max_count;
	float multiplier = 0.90;

	while(true) {
		//set to loop once only, might change the algorithm in the future
		max_count = 0;
		float exp = (multiplier*(float)FSR-counts_expected);
		if (exp<0){
			break;
		}
		float temp_TINT = TINT + pow(2, log((multiplier*(float)FSR-counts_expected))/log(2))*(2.0/720.0);

		if(temp_TINT>max_TINT){
			break;
		}

		AS7343_set_TINT(handle, temp_TINT);

		uint16_t data[18];
		AS7343_enable_spectral_measurement(handle, true);
		AS7343_read_18(handle, data);
		AS7343_enable_spectral_measurement(handle, false);

		for(uint8_t i = 0; i<18; i++){
			if(i == 5 || i == 11 || i == 17) {
				continue;
			}
			if(data[i]>max_count) {
				max_count = data[i];
			}
		}

		if (max_count >= multiplier*0xFFEE) {
			multiplier = multiplier - 0.05;
			continue;
		}
		else {
			TINT = temp_TINT;
		}
		break;
	}
	AS7343_set_gain(handle, currentGain);
	AS7343_set_TINT(handle, TINT);

}
