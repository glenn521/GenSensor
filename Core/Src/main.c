/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sht3x.h"
#include "PASCO2.h"
#include "VEML6031.h"
#include "AS7343.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SHT31_COUNTER 20
#define PASCO2_COUNTER 20
#define VEML6031_COUNTER 20
#define AS7343_COUNTER 20
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

sht3x_handle_t sht31d;
pasco2_handle_t pasco2;
veml6031_handle_t veml6031;
as7343_handle_t as7343;

uint16_t SHT31_counter = SHT31_COUNTER;
bool SHT31_counter_timeout_flag = false;

uint16_t PASCO2_counter = PASCO2_COUNTER;
bool PASCO2_counter_timeout_flag = false;

uint16_t VEML6031_counter = VEML6031_COUNTER;
bool VEML6031_counter_timeout_flag = false;

uint16_t AS7343_counter = AS7343_COUNTER;
bool AS7343_counter_timeout_flag = false;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void scan_I2C_bus(I2C_HandleTypeDef* bus, uint8_t busNo) {
	HAL_StatusTypeDef result;
	char buf[20];

	sprintf(buf, "Scanning Bus %d\r\n", busNo);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 100);
	memset(buf, 0, sizeof(buf));

	for (int i = 0; i<0x7F; i++)
	  {
		  result = HAL_I2C_IsDeviceReady(bus, (uint16_t) (i<<1), 3, 5);
//		  if (result != HAL_OK)
//		  {
//			  sprintf(buf, "--\r\n");
//			  HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 100);
//			  memset(buf, 0, sizeof(buf));
//		  }
		  if (result == HAL_OK)
		  {
			  sprintf(buf, "0x%X\r\n", i);
			  HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 100);
			  memset(buf, 0, sizeof(buf));
		  }
	  }
}
int takeSize(char *Array)
{
    int count=0;
    while(Array[count]!='\n' && count<100)
    {
    	count++;
    }

    return (count+1);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  scan_I2C_bus(&hi2c1, 1);
  scan_I2C_bus(&hi2c3, 3);

  //SHT handler
  sht31d.i2c_handle = &hi2c3;
  sht31d.device_address = (uint16_t)SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW;
  sht3x_init(&sht31d);

  //PASCO2 handler
  pasco2.i2c_handle = &hi2c3;
  pasco2.device_address = (uint16_t)PASCO2_ADDR;
  PASCO2_init(&pasco2);

  //VEML6031 handler
  veml6031.i2c_handle = &hi2c3;
  veml6031.device_address = VEML6031_I2C_ADDRESS;

  //VEML6031 handler
  as7343.i2c_handle = &hi2c3;
  as7343.device_address = AS7343_ADDR_A;
  AS7343_power(&as7343, true);
  AS7343_set_cycle(&as7343, 3);
  AS7343_direct_config_3_chain(&as7343);

//  if(!PASCO2_get_status(&pasco2)) {
//	  char* error = "Error in PASCO2 configuration.\r\n";
//	  HAL_UART_Transmit(&huart1, error, takeSize(error), 100);
//
//	  return;
//  }

  //variables to store data
  float sht31_temp, sht31_humidity, resolution, as7343_TINT;
  uint16_t co2_ppm;
  uint32_t lux;

  uint16_t as7343_channels[18];

  uint32_t uart_buf_len;
  char uart_buf[100];
  HAL_Delay(100);

  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(SHT31_counter_timeout_flag) {
	  	//trigger SHT35 read

	  	SHT31_counter_timeout_flag = 0;

	  	sht3x_read_temperature_and_humidity(&sht31d, &(sht31_temp), &(sht31_humidity) );

	  	uart_buf_len = sprintf(uart_buf, "Temperature: %0.2f and Humidity: %.2f\r\n\n", sht31_temp, sht31_humidity);
	  	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
	  	memset(uart_buf, 0, sizeof(uart_buf));
	  }

	  else if(PASCO2_counter_timeout_flag) {
	  	//trigger PASCO2 read

	  	PASCO2_counter_timeout_flag = 0;

		co2_ppm = PASCO2_get_ppm(&pasco2);

		uart_buf_len = sprintf(uart_buf, "CO2 Concentration (ppm): %d\r\n\n", co2_ppm);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));
	  }
	  else if(VEML6031_counter_timeout_flag) {
	  	//trigger VEML6031 read

		VEML6031_counter_timeout_flag = 0;

		resolution = VEML6031_optimizer(&veml6031);
		lux = VEML6031_read_light(&veml6031)*resolution;

		uart_buf_len = sprintf(uart_buf, "Light Intensity (lux): %d\r\n\n", lux);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));
	  }
	  else if(AS7343_counter_timeout_flag) {
	  	//trigger VEML6031 read

		AS7343_counter_timeout_flag = 0;

		AS7343_optimizer(&as7343, 1000.0);
		HAL_Delay(20);
		AS7343_read_18(&as7343, as7343_channels);

		uart_buf_len = sprintf(uart_buf, "AS7343 F1: 0x%x - 0d%d\r\n", as7343_channels[12], as7343_channels[12]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F2: 0x%x - 0d%d\r\n", as7343_channels[6], as7343_channels[6]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 FZ: 0x%x - 0d%d\r\n", as7343_channels[0], as7343_channels[0]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F3: 0x%x - 0d%d\r\n", as7343_channels[7], as7343_channels[7]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F4: 0x%x - 0d%d\r\n", as7343_channels[8], as7343_channels[8]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 FY: 0x%x - 0d%d\r\n", as7343_channels[1], as7343_channels[1]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F5: 0x%x - 0d%d\r\n", as7343_channels[15], as7343_channels[15]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 FXL: 0x%x - 0d%d\r\n", as7343_channels[2], as7343_channels[2]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F6: 0x%x - 0d%d\r\n", as7343_channels[9], as7343_channels[9]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F7: 0x%x - 0d%d\r\n", as7343_channels[13], as7343_channels[13]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 F8: 0x%x - 0d%d\r\n", as7343_channels[14], as7343_channels[14]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uart_buf_len = sprintf(uart_buf, "AS7343 NIR: 0x%x - 0d%d\r\n", as7343_channels[3], as7343_channels[3]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uint16_t VIS = (as7343_channels[4]<<1)+(as7343_channels[10]<<1);
		uart_buf_len = sprintf(uart_buf, "AS7343 VIS: 0x%x - 0d%d\r\n", as7343_channels[4], as7343_channels[4]);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		uint8_t as7343_AGAIN = AS7343_get_gain(&as7343);
		uart_buf_len = sprintf(uart_buf, "AS7343 GAIN: %d\r\n", as7343_AGAIN);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));

		as7343_TINT = AS7343_get_TINT(&as7343);
		uart_buf_len = sprintf(uart_buf, "AS7343 TINT: %f\r\n\n\n", as7343_TINT);
		HAL_UART_Transmit(&huart1, uart_buf, uart_buf_len, 100);
		memset(uart_buf, 0, sizeof(uart_buf));
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 124;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 63998;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	/*
	 * Calls when Timer period is reached T = 0.5s
	 */
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

	if(--SHT31_counter == 0) {
		//SHT3x Handler
		SHT31_counter_timeout_flag = true;
		SHT31_counter = SHT31_COUNTER;
	}
	if(--PASCO2_counter == 0) {
		//PASCO2 Handler
		PASCO2_counter_timeout_flag = true;
		PASCO2_counter = PASCO2_COUNTER;
	}
	if(--VEML6031_counter == 0) {
		//VEML6031 Handler
		VEML6031_counter_timeout_flag = true;
		VEML6031_counter = VEML6031_COUNTER;
	}
	if(--AS7343_counter == 0) {
		//AS7343 Handler
		AS7343_counter_timeout_flag = true;
		AS7343_counter = AS7343_COUNTER;
	}
//	if(--MLX90614_counter == 0) {
//		//MLX90614 Handler
//		MLX90614_counter_timeout_flag = true;
//		MLX90614_counter = MLX90614_COUNTER;
//	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
