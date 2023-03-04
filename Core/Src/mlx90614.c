/*
 * mlx90614.c
 *
 *  Created on: Oct 16, 2022
 *      Author: Yousif
 */

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "mlx90614.h"

I2C_HandleTypeDef 	*MLX90614_i2c;
extern UART_HandleTypeDef huart2;
char uart_buf[128] = {'\0'};

uint8_t crc8 (uint8_t inCrc, uint8_t inData) {
	uint8_t i;
	uint8_t data;

	data = inCrc ^ inData;

	for(i = 0; i<8; i++) {
		if((data & 0x80) != 0){
			data <<= 1;
			data ^= 0x07;
		}
		else {
			data <<= 1;
		}
	}
	return data;
}

void MLX90614_Init(I2C_HandleTypeDef *hi2c){
	/*
	 * Initialize MLX90614 I2C and registers
	 */

	MLX90614_i2c = hi2c;

	uint16_t data = 0;

	sprintf(uart_buf, "Starting Initialization of MLX_90614 IR Sensor:\r\n");
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
	HAL_Delay(100);
	memset(uart_buf, 0, sizeof(uart_buf));

	MLX90614_WriteReg(MLX90614_TOMIN, 0x6AB3);
	MLX90614_ReadReg(MLX90614_TOMIN, &data);
    sprintf(uart_buf, "ToMin: 0x%X\r\n", data);
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
    HAL_Delay(100);

    MLX90614_WriteReg(MLX90614_TOMAX, 0x7E3B);
    MLX90614_ReadReg(MLX90614_TOMAX, &data);
    sprintf(uart_buf, "ToMax: 0x%X\r\n", data);
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
    HAL_Delay(100);

//    MLX90614_WriteReg(MLX90614_PWMCTRL, 0x);
    MLX90614_ReadReg(MLX90614_PWMCTRL, &data);
    sprintf(uart_buf, "PWMCTRL: 0x%X\r\n", data);
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
    HAL_Delay(100);

//    MLX90614_WriteReg(MLX90614_TARANGE, 0x);
    MLX90614_ReadReg(MLX90614_TARANGE, &data);
    sprintf(uart_buf, "TA Range: 0x%X\r\n", data);
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
    HAL_Delay(100);

    MLX90614_WriteReg(MLX90614_EMISSIVITY, 0xFAE1);
    MLX90614_ReadReg(MLX90614_EMISSIVITY, &data);
    sprintf(uart_buf, "EMISSIVITY: 0x%X\r\n", data);
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
    HAL_Delay(100);

    MLX90614_ReadReg(MLX90614_CFG1, &data);
    sprintf(uart_buf, "CFG1: 0x%X\r\n", data);
    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
    HAL_Delay(100);

//    MLX90614_WriteReg(MLX90614_CFG1, 0xB7C0);
//    MLX90614_ReadReg(MLX90614_CFG1, &data);
//    sprintf(uart_buf, "CFG1: 0x%X\r\n", data);
//    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
//    HAL_Delay(100);
//
//    MLX90614_WriteReg(MLX90614_PWMCTRL, 0x1405);
//    MLX90614_ReadReg(MLX90614_PWMCTRL, &data);
//    sprintf(uart_buf, "PWMCTRL: 0x%X\r\n", data);
//    HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
//    HAL_Delay(100);

    sprintf(uart_buf, "MLX_90614 IR Sensor Initialization Finished.\r\n");
	HAL_UART_Transmit(&huart2, uart_buf, sizeof(uart_buf), 100);
	HAL_Delay(100);
}

void MLX90614_WriteReg(uint8_t regAddr, uint16_t data) {
	/*
	 *
	 */

	uint8_t i2cdata[4];
	uint8_t crc;
	uint8_t lsb = data & 0x00FF;
	uint8_t msb = (data >> 8);

	crc = crc8(0x0, MLX90614_DEFAULT_SA<<1);
	crc = crc8(crc, regAddr);
	crc = crc8(crc, 0x00);
	crc = crc8(crc, 0x00);

	i2cdata[0] = regAddr;
	i2cdata[1] = 0x00;
	i2cdata[2] = 0x00;
	i2cdata[3] = crc;

	HAL_I2C_Master_Transmit(MLX90614_i2c, (MLX90614_DEFAULT_SA << 1), i2cdata, 4, 0xFFFF);
	HAL_Delay(10);

	crc = crc8(0x0, MLX90614_DEFAULT_SA<<1);
	crc = crc8(crc, regAddr);
	crc = crc8(crc, lsb);
	crc = crc8(crc, msb);

	i2cdata[0] = regAddr;
	i2cdata[1] = lsb;
	i2cdata[2] = msb;
	i2cdata[3] = crc;

	HAL_I2C_Master_Transmit(MLX90614_i2c, (MLX90614_DEFAULT_SA << 1), i2cdata, 4, 0xFFFF);
	HAL_Delay(10);

	return;
}

bool MLX90614_ReadReg(uint8_t regAddr, uint16_t * dest) {
	/*
	 *
	 */

	uint8_t in_buff[3], crc;

	HAL_I2C_Mem_Read(MLX90614_i2c, (MLX90614_DEFAULT_SA<<1), regAddr, 1, in_buff, 3, 100);

	crc = crc8(0x0, MLX90614_DEFAULT_SA<<1);
	crc = crc8(crc, regAddr);
	crc = crc8(crc, (MLX90614_DEFAULT_SA<<1) + 1);
	crc = crc8(crc, in_buff[0]);
	crc = crc8(crc, in_buff[1]);

	if (crc == in_buff[2]) {
		*dest = ((uint16_t)in_buff[1] << 8) | in_buff[0];
		return 1;
	}

	else {
		return 0;
	}
}

float MLX90614_ReadTObj1() {
	uint16_t data;

	if(MLX90614_ReadReg(MLX90614_TOBJ1, &data)) {
		if(data & 0x8000) {
			return -1.0;
		}
		return MLX90614_CalcTemp(data);
	}
	return -1.00;
}

float MLX90614_ReadTObj2() {
	uint16_t data;

	if(MLX90614_ReadReg(MLX90614_TOBJ2, &data)) {
		if(data & 0x8000) {
			return -1.0;
		}
		return MLX90614_CalcTemp(data);
	}
	return -1.00;
}

float MLX90614_ReadTAmb() {
	uint16_t data;

	if(MLX90614_ReadReg(MLX90614_TAMB, &data)) {
		return MLX90614_CalcTemp(data);
	}
	return -1.00;
}

float MLX90614_CalcTemp(uint16_t rawTemp) {
	float temp;

	temp = (int16_t)rawTemp*0.02 - 273.15; //Celsius

	return temp;
	//TO DO: Implement other units
}

