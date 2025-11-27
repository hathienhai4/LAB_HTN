/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "software_timer.h"
#include "led_7seg.h"
#include "button.h"
#include "lcd.h"
#include "picture.h"
#include "ds3231.h"
#include "uart.h"
#include "sensor.h"
#include "buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INIT				0

#define MODE_DISPLAY_TIME 	2
#define MODE_SET_TIME 		3
#define MODE_SET_ALARM 		4
#define MODE_SET_UART		5

#define PARAM_HOUR 			10
#define PARAM_MIN 			11
#define PARAM_SEC 			12
#define PARAM_DATE 			13
#define PARAM_MONTH 		14
#define PARAM_YEAR 			15
#define PARAM_DAY 			16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t count_led_debug = 0;

// FSM variables
uint8_t setting_param = PARAM_HOUR;
uint8_t blink_state = 0;
uint8_t blink_counter = 0;

// Temporary time values for setting
uint8_t temp_hour = 0;
uint8_t temp_min = 0;
uint8_t temp_sec = 0;
uint8_t temp_date = 0;
uint8_t temp_month = 0;
uint8_t temp_year = 0;
uint8_t temp_day = 0;

// Alarm values
uint8_t alarm_hour = 0;
uint8_t alarm_min = 0;
uint8_t alarm_sec = 0;
uint8_t alarm_triggered = 0;

// Button press duration tracking
uint16_t button_up_duration = 0;
uint16_t auto_increment_counter = 0;
// Request duration tracking
uint16_t request_duration = 0;
uint16_t request_counter = 0;
uint8_t request_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
void system_init();
void test_LedDebug();
void displayTime();
void updateTime();
void displayModeIndicator();
void displayAlarmTime();
void checkAlarm();
void displayTime();
void fsm_clock();
void test_Uart();
void handle_Uart();
void set_time_Uart();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  system_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_Clear(BLACK);
  updateTime();
  while (1)
  {
	  while(!flag_timer2);
	  flag_timer2 = 0;
	  button_Scan();
	  ds3231_ReadTime();
	  test_Esp();
	  lightProcess();
	  test_LedDebug();
//	  displayTime();
//	  test_Uart();
//	  handle_Uart();
//	  fsm_clock();
	  tempProcess();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 5;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 840-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DEBUG_LED_Pin|OUTPUT_Y0_Pin|OUTPUT_Y1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSMC_RES_GPIO_Port, FSMC_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP12_BUSY_GPIO_Port, ESP12_BUSY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_LATCH_GPIO_Port, LD_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSMC_BLK_GPIO_Port, FSMC_BLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_Pin OUTPUT_Y0_Pin OUTPUT_Y1_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin|OUTPUT_Y0_Pin|OUTPUT_Y1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : FSMC_RES_Pin */
  GPIO_InitStruct.Pin = FSMC_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSMC_RES_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP12_PWR_Pin */
  GPIO_InitStruct.Pin = ESP12_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP12_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP12_BUSY_Pin */
  GPIO_InitStruct.Pin = ESP12_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESP12_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_X0_Pin INPUT_X1_Pin */
  GPIO_InitStruct.Pin = INPUT_X0_Pin|INPUT_X1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_X2_Pin INPUT_X3_Pin */
  GPIO_InitStruct.Pin = INPUT_X2_Pin|INPUT_X3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_LATCH_Pin */
  GPIO_InitStruct.Pin = LD_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_LATCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FSMC_BLK_Pin */
  GPIO_InitStruct.Pin = FSMC_BLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSMC_BLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_LOAD_Pin */
  GPIO_InitStruct.Pin = BTN_LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN_LOAD_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NORSRAM_TimingTypeDef ExtTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 0xf;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 60;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 8;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 9;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 17;
  ExtTiming.AccessMode = FSMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void system_init(){
	  HAL_GPIO_WritePin(OUTPUT_Y0_GPIO_Port, OUTPUT_Y0_Pin, 0);
	  HAL_GPIO_WritePin(OUTPUT_Y1_GPIO_Port, OUTPUT_Y1_Pin, 0);
	  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, 0);
	  timer_init();
	  led7_init();
	  button_init();
	  lcd_init();
	  ds3231_init();
	  uart_init_rs232();
	  uart_init_esp();
	  setTimer2(50);
}

void test_LedDebug(){
	count_led_debug = (count_led_debug + 1)%20;
	if(count_led_debug == 0){
		HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
	}
}

void test_7seg(){
	led7_SetDigit(0, 0, 0);
	led7_SetDigit(5, 1, 0);
	led7_SetDigit(4, 2, 0);
	led7_SetDigit(7, 3, 0);
}
void test_button(){
	for(int i = 0; i < 16; i++){
		if(button_count[i] == 1){
			led7_SetDigit(i/10, 2, 0);
			led7_SetDigit(i%10, 3, 0);
		}
	}
}

void updateTime(){
	ds3231_Write(ADDRESS_YEAR, 25);
	ds3231_Write(ADDRESS_MONTH, 11);
	ds3231_Write(ADDRESS_DATE, 03);
	ds3231_Write(ADDRESS_DAY, 2);
	ds3231_Write(ADDRESS_HOUR, 17);
	ds3231_Write(ADDRESS_MIN, 45);
	ds3231_Write(ADDRESS_SEC, 30);
}

uint8_t isButtonUp()
{
    if (button_count[3] == 1)
        return 1;
    else
        return 0;
}
uint8_t isButtonDown()
{
    if (button_count[7] == 1)
        return 1;
    else
        return 0;
}

uint8_t state = INIT;
char *mode_str = "";
void displayModeIndicator() {
    lcd_Fill(0, 0, 240, 20, BLACK);
    switch(state) {
        case MODE_DISPLAY_TIME:
            lcd_ShowStr(10, 5, "MODE: DISPLAY TIME", WHITE, BLACK, 16, 0);
            break;
        case MODE_SET_TIME:
            lcd_ShowStr(10, 5, "MODE: SET TIME", CYAN, BLACK, 16, 0);
            break;
        case MODE_SET_ALARM:
            lcd_ShowStr(10, 5, "MODE: SET ALARM", MAGENTA, BLACK, 16, 0);
            break;
        case MODE_SET_UART:
            lcd_ShowStr(10, 5, mode_str, MAGENTA, BLACK, 16, 0);
            break;
    }
}

void checkAlarm() {
    if(alarm_hour == ds3231_hours && alarm_min == ds3231_min && alarm_sec == ds3231_sec)
        if(!alarm_triggered)
            alarm_triggered = 1;

    if(alarm_triggered) {
        lcd_Fill(0, 180, 240, 220, RED);
        lcd_ShowStr(40, 190, "*** ALARM ***", WHITE, RED, 24, 0);

        for(int i = 0; i < 16; i++) {
            if(button_count[i] == 1) {
                alarm_triggered = 0;
                lcd_Fill(0, 180, 240, 220, BLACK);
                break;
            }
        }
    }
}

void displayAlarmTime() {
    lcd_ShowStr(10, 260, "ALARM:", YELLOW, BLACK, 16, 0);
    lcd_ShowIntNum(70, 260, alarm_hour, 2, YELLOW, BLACK, 16);
    lcd_ShowIntNum(100, 260, alarm_min, 2, YELLOW, BLACK, 16);
    lcd_ShowIntNum(130, 260, alarm_sec, 2, YELLOW, BLACK, 16);
}

void displayTime(){
	blink_counter++;
	if(blink_counter >= 10) {
		blink_counter = 0;
		blink_state = !blink_state;
	}

	uint16_t hour_color = GREEN, min_color = GREEN, sec_color = GREEN;
	uint16_t date_color = YELLOW, month_color = YELLOW, year_color = YELLOW, day_color = YELLOW;

	if((state == MODE_SET_TIME || state == MODE_SET_ALARM || state == MODE_SET_UART) && blink_state) {
		switch(setting_param) {
			case PARAM_HOUR: hour_color = BLACK; break;
			case PARAM_MIN: min_color = BLACK; break;
			case PARAM_SEC: sec_color = BLACK; break;
			case PARAM_DATE: date_color = BLACK; break;
			case PARAM_MONTH: month_color = BLACK; break;
			case PARAM_YEAR: year_color = BLACK; break;
			case PARAM_DAY: day_color = BLACK; break;
		}
	}

	// Display based on mode
	if(state == MODE_SET_TIME || state == MODE_SET_UART) {
		lcd_ShowIntNum(70, 100, temp_hour, 2, hour_color, BLACK, 24);
		lcd_ShowIntNum(110, 100, temp_min, 2, min_color, BLACK, 24);
		lcd_ShowIntNum(150, 100, temp_sec, 2, sec_color, BLACK, 24);

		lcd_ShowIntNum(20, 130, temp_day, 2, day_color, BLACK, 24);
		lcd_ShowIntNum(70, 130, temp_date, 2, date_color, BLACK, 24);
		lcd_ShowIntNum(110, 130, temp_month, 2, month_color, BLACK, 24);
		lcd_ShowIntNum(150, 130, temp_year, 2, year_color, BLACK, 24);
	} else if(state == MODE_SET_ALARM) {
		lcd_ShowStr(10, 100, "SET ALARM:", MAGENTA, BLACK, 24, 0);
		lcd_ShowIntNum(70, 130, alarm_hour, 2, hour_color, BLACK, 24);
		lcd_ShowIntNum(110, 130, alarm_min, 2, min_color, BLACK, 24);
		lcd_ShowIntNum(150, 130, alarm_sec, 2, sec_color, BLACK, 24);
	} else {
		lcd_ShowIntNum(70, 100, ds3231_hours, 2, hour_color, BLACK, 24);
		lcd_ShowIntNum(110, 100, ds3231_min, 2, min_color, BLACK, 24);
		lcd_ShowIntNum(150, 100, ds3231_sec, 2, sec_color, BLACK, 24);

		lcd_ShowIntNum(20, 130, ds3231_day, 2, day_color, BLACK, 24);
		lcd_ShowIntNum(70, 130, ds3231_date, 2, date_color, BLACK, 24);
		lcd_ShowIntNum(110, 130, ds3231_month, 2, month_color, BLACK, 24);
		lcd_ShowIntNum(150, 130, ds3231_year, 2, year_color, BLACK, 24);

		displayAlarmTime();
		checkAlarm();
	}
}

//void incrementParameter() {
//	switch(setting_param) {
//		case PARAM_HOUR:
//			if(state == MODE_SET_TIME) temp_hour = (temp_hour + 1) % 24;
//			else alarm_hour = (alarm_hour + 1) % 24;
//			break;
//		case PARAM_MIN:
//			if(state == MODE_SET_TIME) temp_min = (temp_min + 1) % 60;
//			else alarm_min = (alarm_min + 1) % 60;
//			break;
//		case PARAM_SEC:
//			if(state == MODE_SET_TIME) temp_sec = (temp_sec + 1) % 60;
//			else alarm_sec = (alarm_sec + 1) % 60;
//			break;
//		case PARAM_DATE:
//			temp_date = (temp_date % 31) + 1;
//			break;
//		case PARAM_MONTH:
//			temp_month = (temp_month % 12) + 1;
//			break;
//		case PARAM_YEAR:
//			temp_year = (temp_year + 1) % 100;
//			break;
//		case PARAM_DAY:
//			temp_day = (temp_day % 7) + 1;
//			break;
//	}
//}

void handleIncrementButton() {
	if(button_count[3] >= 1) {
		button_up_duration++;
		if(button_count[3] == 1) {	// Single press - increment once
			switch(setting_param) {
				case PARAM_HOUR:
					if(state == MODE_SET_TIME) temp_hour = (temp_hour + 1) % 24;
					else alarm_hour = (alarm_hour + 1) % 24;
					break;
				case PARAM_MIN:
					if(state == MODE_SET_TIME) temp_min = (temp_min + 1) % 60;
					else alarm_min = (alarm_min + 1) % 60;
					break;
				case PARAM_SEC:
					if(state == MODE_SET_TIME) temp_sec = (temp_sec + 1) % 60;
					else alarm_sec = (alarm_sec + 1) % 60;
					break;
				case PARAM_DATE:
					temp_date = (temp_date % 31) + 1;
					break;
				case PARAM_MONTH:
					temp_month = (temp_month % 12) + 1;
					break;
				case PARAM_YEAR:
					temp_year = (temp_year + 1) % 100;
					break;
				case PARAM_DAY:
					temp_day = (temp_day % 7) + 1;
					break;
			}
		}
		else if(button_up_duration >= 40) { // Held for 2 seconds (40 cycles * 50ms)
			auto_increment_counter++;
			if(auto_increment_counter >= 4) { // Auto increment every 200ms (4 cycles)
				auto_increment_counter = 0;
				switch(setting_param) {
					case PARAM_HOUR:
						if(state == MODE_SET_TIME) temp_hour = (temp_hour + 1) % 24;
						else alarm_hour = (alarm_hour + 1) % 24;
						break;
					case PARAM_MIN:
						if(state == MODE_SET_TIME) temp_min = (temp_min + 1) % 60;
						else alarm_min = (alarm_min + 1) % 60;
						break;
					case PARAM_SEC:
						if(state == MODE_SET_TIME) temp_sec = (temp_sec + 1) % 60;
						else alarm_sec = (alarm_sec + 1) % 60;
						break;
					case PARAM_DATE:
						temp_date = (temp_date % 31) + 1;
						break;
					case PARAM_MONTH:
						temp_month = (temp_month % 12) + 1;
						break;
					case PARAM_YEAR:
						temp_year = (temp_year + 1) % 100;
						break;
					case PARAM_DAY:
						temp_day = (temp_day % 7) + 1;
						break;
				}
			}
		}
	} else {
		button_up_duration = 0;
		auto_increment_counter = 0;
	}
}
void handleResendRequest() {
	request_duration ++;
	if(request_duration >= 100){ // wait for 5 seconds (100 cycles * 50ms)
		request_counter ++;
		request_duration = 0;
		request_flag = 1;
	}
	if(request_counter > 3){
		request_counter = 0;
		request_duration = 0;
		request_flag = 0;
		state = MODE_DISPLAY_TIME;
		setting_param = PARAM_HOUR;
		lcd_Clear(BLACK);
		uart_Rs232SendString("Finish setting !");
	}
}
void fsm_clock() {
	static uint8_t last_state = 0xFF;

	switch (state) {
	case INIT:
		lcd_Clear(BLACK);
		displayModeIndicator();
		state = MODE_DISPLAY_TIME;
		break;
	case MODE_DISPLAY_TIME:
		if(isButtonDown()) {
			state = MODE_SET_TIME;
			setting_param = PARAM_HOUR;
			lcd_Clear(BLACK);

			temp_hour = ds3231_hours;
			temp_min = ds3231_min;
			temp_sec = ds3231_sec;
			temp_date = ds3231_date;
			temp_month = ds3231_month;
			temp_year = ds3231_year;
			temp_day = ds3231_day;
			last_state = 0xFF;
		}

		if(last_state != state) {
			displayModeIndicator();
			last_state = state;
		}

		displayTime();
		break;
	case MODE_SET_TIME:
		if(isButtonDown()) {
			state = MODE_SET_ALARM;
			setting_param = PARAM_HOUR;
			lcd_Clear(BLACK);
			last_state = 0xFF;
		}

		if(last_state != state) {
			displayModeIndicator();
			last_state = state;
		}

		handleIncrementButton();

		if(button_count[12] == 1) {
			setting_param++;

			if(setting_param > PARAM_DAY) {
				ds3231_Write(ADDRESS_HOUR, temp_hour);
				ds3231_Write(ADDRESS_MIN, temp_min);
				ds3231_Write(ADDRESS_SEC, temp_sec);
				ds3231_Write(ADDRESS_DATE, temp_date);
				ds3231_Write(ADDRESS_MONTH, temp_month);
				ds3231_Write(ADDRESS_YEAR, temp_year);
				ds3231_Write(ADDRESS_DAY, temp_day);

				state = MODE_DISPLAY_TIME;
				setting_param = PARAM_HOUR;
				lcd_Clear(BLACK);
				last_state = 0xFF;
			}
		}

		displayTime();
		break;

	case MODE_SET_ALARM:

		if(last_state != state) {
			displayModeIndicator();
			last_state = state;
		}

		handleIncrementButton();

		if(button_count[12] == 1) {
			setting_param++;

			if(setting_param > PARAM_SEC) {
				state = MODE_DISPLAY_TIME;
				setting_param = PARAM_HOUR;
				lcd_Clear(BLACK);
				last_state = 0xFF;
			}
		}

		displayTime();

		if(isButtonDown()) {
			state = MODE_SET_UART;
			setting_param = PARAM_HOUR;
			lcd_Clear(BLACK);
			last_state = 0xFF;
		}
		break;
	case MODE_SET_UART:
		if(isButtonDown()) {
			state = MODE_DISPLAY_TIME;
			setting_param = PARAM_HOUR;
			lcd_Clear(BLACK);
			last_state = 0xFF;
		}

		if(last_state != state) {
			mode_str = "MODE: UART updating hours";
			uart_Rs232SendString("Type hour value: ");
			last_state = state;
		}
		handleResendRequest();
		displayModeIndicator();
		set_time_Uart();
		displayTime();
		if(isButtonDown()) {
			state = MODE_DISPLAY_TIME;
			setting_param = PARAM_HOUR;
			lcd_Clear(BLACK);
			last_state = 0xFF;
		}
		break;

	}
}
void test_Uart(){
	if(button_count[12] == 1){
		uart_Rs232SendNum(ds3231_hours);
		uart_Rs232SendString(":");
		uart_Rs232SendNum(ds3231_min);
		uart_Rs232SendString(":");
		uart_Rs232SendNum(ds3231_sec);
		uart_Rs232SendString("\n");
	}
}
int get_char_Uart(void) {
	if(bf_head == bf_tail) return -1;
    uint8_t c = buffer[bf_tail];
    bf_tail = (bf_tail + 1) % BUFFER_SIZE;
    return c;
}
void handle_Uart(){
	static uint8_t cmd [32];
	static uint8_t idx = 0;
    int c;
    uint8_t ch;
    if (uart_flag) {
    	while((c = get_char_Uart()) != -1){
    		ch = (uint8_t)c;
			HAL_UART_Transmit(&huart1, &ch, 1, 10);
			if (ch == '\n') {
				cmd[idx] = '\0';
				idx = 0;
//				uart_Rs232SendString((uint8_t*)"\n");
				if (strcmp((char*)cmd, "hour") == 0) {
					uart_Rs232SendString((uint8_t*)"Hour:");
					uart_Rs232SendNum(ds3231_hours);
					uart_Rs232SendString((uint8_t*)"\n");
				} else if (strcmp((char*)cmd, "minute") == 0) {
					uart_Rs232SendString((uint8_t*)"Minute:");
					uart_Rs232SendNum(ds3231_min);
					uart_Rs232SendString((uint8_t*)"\n");
				} else if (strcmp((char*)cmd, "second") == 0) {
					uart_Rs232SendString((uint8_t*)"Second:");
					uart_Rs232SendNum(ds3231_sec);
					uart_Rs232SendString((uint8_t*)"\n");
				} else {
					uart_Rs232SendString((uint8_t*)"Unknown command!!!");
					uart_Rs232SendString((uint8_t*)"\n");
				}
			}
			else if(ch == '\r') {
				continue;
			}
			else if(ch == 0x08 || ch == 0x7F) {
				idx = (idx > 0)? idx - 1 : idx;
			}
			else if (idx < sizeof(cmd) - 1) {
				cmd[idx] = ch;
				idx ++;
			}
    	}
        uart_flag = 0; // reset c�? sau khi xử lý xong
    }
}
void set_time_Uart(){
	static uint8_t cmd [32];
	static uint8_t idx = 0;
    int c;
    int time = 0;
    uint8_t* endptr;
    uint8_t ch;
	switch(setting_param) {
		case PARAM_HOUR:
			mode_str = "MODE: UART updating hours";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType hour value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
		case PARAM_MIN:
			mode_str = "MODE: UART updating minute";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType minute value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
		case PARAM_SEC:
			mode_str = "MODE: UART updating second";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType second value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
		case PARAM_DATE:
			mode_str = "MODE: UART updating date";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType date value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
		case PARAM_MONTH:
			mode_str = "MODE: UART updating month";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType month value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
		case PARAM_YEAR:
			mode_str = "MODE: UART updating year";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType year value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
		case PARAM_DAY:
			mode_str = "MODE: UART updating day";
			if(request_flag != 0) {
				uart_Rs232SendString("\nType day value: ");
				request_flag = 0;
				idx = 0;
				cmd[idx] = '\0';
			}
			break;
	}

    if (uart_flag) {
    	while((c = get_char_Uart()) != -1){
    		ch = (uint8_t)c;
			HAL_UART_Transmit(&huart1, &ch, 1, 10);
			if (ch == '\n') {
				cmd[idx] = '\0';
				idx = 0;
//				uart_Rs232SendString((uint8_t*)"\n");
				time = strtol((char*)cmd, &endptr, 10);
				switch(setting_param) {
					case PARAM_HOUR:
//						mode_str = "MODE: UART updating hours";
						if (*endptr != '\0' || time > 23 || time < 0) {
							// có ký tự không hợp lệ
							uart_Rs232SendString((uint8_t*)"Invalid value!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type hour value: ");
						}
						else if(cmd[0] == '\0'){
							setting_param ++;
							uart_Rs232SendString("Type minute value: ");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Hour is set to: ");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_hour = time;
							setting_param ++;
							uart_Rs232SendString("Type minute value: ");
						}
						break;
					case PARAM_MIN:
//						mode_str = "MODE: UART updating minute";
						if(*endptr != '\0' || time > 59 || time < 0){
							uart_Rs232SendString((uint8_t*)"Invalid number!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type minute value: ");
						}
						else if(cmd[0] == '\0'){
							setting_param ++;
							uart_Rs232SendString("Type second value: ");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Minute is set to: ");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_min = time;
							setting_param ++;
							uart_Rs232SendString("Type second value: ");
						}
						break;
					case PARAM_SEC:
//						mode_str = "MODE: UART updating second";
						if(*endptr != '\0' || time > 59 || time < 0){
							uart_Rs232SendString((uint8_t*)"Invalid number!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type second value: ");
						}
						else if(cmd[0] == '\0'){
							setting_param ++;
							uart_Rs232SendString("Type date value: ");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Second is set to: ");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_sec = time;
							setting_param ++;
							uart_Rs232SendString("Type date value: ");
						}
						break;
					case PARAM_DATE:
//						mode_str = "MODE: UART updating date";
						if(*endptr != '\0' || time > 31 || time < 0){
							uart_Rs232SendString((uint8_t*)"Invalid number!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type date value: ");
						}
						else if(cmd[0] == '\0'){
							setting_param ++;
							uart_Rs232SendString("Type month value: ");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Date is set to: ");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_date = time;
							setting_param ++;
							uart_Rs232SendString("Type month value: ");
						}
						break;
					case PARAM_MONTH:
//						mode_str = "MODE: UART updating month";
						if(*endptr != '\0' || time > 12 || time < 0){
							uart_Rs232SendString((uint8_t*)"Invalid number!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type month value: ");
						}
						else if(cmd[0] == '\0'){
							setting_param ++;
							uart_Rs232SendString("Type year value: ");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Month is set to: ");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_month = time;
							setting_param ++;
							uart_Rs232SendString("Type year value: ");
						}
						break;
					case PARAM_YEAR:
//						mode_str = "MODE: UART updating year";
						if(*endptr != '\0' || time > 99 || time < 0){
							uart_Rs232SendString((uint8_t*)"Invalid number!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type year value: ");
						}
						else if(cmd[0] == '\0'){
							setting_param ++;
							uart_Rs232SendString("Type day value: ");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Year is set to :");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_year = time;
							setting_param ++;
							uart_Rs232SendString("Type day value: ");
						}
						break;
					case PARAM_DAY:
//						mode_str = "MODE: UART updating day";
						if(*endptr != '\0' || time > 7 || time < 0){
							uart_Rs232SendString((uint8_t*)"Invalid number!!!");
							uart_Rs232SendString((uint8_t*)"\n");
							uart_Rs232SendString("Type day value: ");
						}
						else if(cmd[0] == '\0'){
							ds3231_Write(ADDRESS_HOUR, temp_hour);
							ds3231_Write(ADDRESS_MIN, temp_min);
							ds3231_Write(ADDRESS_SEC, temp_sec);
							ds3231_Write(ADDRESS_DATE, temp_date);
							ds3231_Write(ADDRESS_MONTH, temp_month);
							ds3231_Write(ADDRESS_YEAR, temp_year);
							ds3231_Write(ADDRESS_DAY, temp_day);

							state = MODE_DISPLAY_TIME;
							setting_param = PARAM_HOUR;
							lcd_Clear(BLACK);
							uart_Rs232SendString("Finish setting !");
						}
						else{
							uart_Rs232SendString((uint8_t*)"Day is set to: ");
							uart_Rs232SendNum(time);
							uart_Rs232SendString((uint8_t*)"\n");
							temp_day = time;
							ds3231_Write(ADDRESS_HOUR, temp_hour);
							ds3231_Write(ADDRESS_MIN, temp_min);
							ds3231_Write(ADDRESS_SEC, temp_sec);
							ds3231_Write(ADDRESS_DATE, temp_date);
							ds3231_Write(ADDRESS_MONTH, temp_month);
							ds3231_Write(ADDRESS_YEAR, temp_year);
							ds3231_Write(ADDRESS_DAY, temp_day);

							state = MODE_DISPLAY_TIME;
							setting_param = PARAM_HOUR;
							lcd_Clear(BLACK);
							uart_Rs232SendString("Finish setting !");
						}
						break;
				}
			}
			else if(ch == '\r') {
				continue;
			}
			else if(ch == 0x08 || ch == 0x7F) {
				idx = (idx > 0)? idx - 1 : idx;
			}
			else if (idx < sizeof(cmd) - 1) {
				time = strtol((char*)cmd, &endptr, 10);
				if (*endptr != '\0') {
				    // có ký tự không hợp lệ
				}
				cmd[idx] = ch;
				idx ++;
			}
    	}
    	request_counter = 0;
    	request_flag = 0;
    	request_duration = 0;
        uart_flag = 0; // reset c�? sau khi xử lý xong
    }

}
uint8_t count_adc = 0;

// Calculate power consumption in mW
float getPowerConsumption() {
	return sensor_GetVoltage() * sensor_GetCurrent();
}

// Get humidity from potentiometer (0-100%)
float getHumidity() {
	uint16_t pot_value = sensor_GetPotentiometer();
	// Map ADC value (0-4095) to humidity (0-100%)
	return (pot_value * 100.0f) / 4095.0f;
}

// Classify light level
const char* getLightLevel() {
	uint16_t light = sensor_GetLight();
	// Threshold at 2000 (adjustable based on sensor)
	if (light > 2000) {
		return "Strong";
	} else {
		return "Weak";
	}
}

void test_Adc(){
	count_adc = (count_adc + 1)%20;
	if(count_adc == 0){
		sensor_Read();

		// Display environmental monitoring data
		float power = getPowerConsumption();
		float humidity = getHumidity();
		const char* light_level = getLightLevel();
		float temperature = sensor_GetTemperature();

		// Display on LCD
		lcd_ShowStr(10, 40, "Power:", RED, BLACK, 16, 0);
		lcd_ShowFloatNum(100, 40, power, 4, RED, BLACK, 16);
		lcd_ShowStr(180, 40, "mW", RED, BLACK, 16, 0);

		lcd_ShowStr(10, 60, "Light:", RED, BLACK, 16, 0);
		lcd_ShowStr(100, 60, "      ", BLACK, BLACK, 16, 0);
		lcd_ShowStr(100, 60, (uint8_t*)light_level, RED, BLACK, 16, 0);

		lcd_ShowStr(10, 80, "Temp:", RED, BLACK, 16, 0);
		lcd_ShowFloatNum(100, 80, temperature, 4, RED, BLACK, 16);
		lcd_ShowStr(180, 80, "C", RED, BLACK, 16, 0);

		lcd_ShowStr(10, 100, "Humidity:", RED, BLACK, 16, 0);
		lcd_ShowFloatNum(120, 100, humidity, 4, RED, BLACK, 16);
		lcd_ShowStr(180, 100, "%", RED, BLACK, 16, 0);

		// Check alarm condition
		if (humidity > 70.0f) {
			lcd_ShowStr(10, 120, "ALARM: High Humidity!", RED, BLACK, 16, 0);
		} else {
			lcd_ShowStr(10, 120, "                      ", BLACK, BLACK, 16, 0);
		}
	}
}
void tempProcess() {
    count_adc = (count_adc + 1) % 600;

    if (count_adc == 0) {

        // 1. Đọc cảm biến
        sensor_Read();

        // 2. Lấy giá trị nhiệt độ
        float temperature = sensor_GetTemperature();

        // 3. Hiển thị lên LCD
        lcd_ShowStr(10, 80, "Temp:", RED, BLACK, 16, 0);
        lcd_ShowFloatNum(100, 80, temperature, 4, RED, BLACK, 16);
        lcd_ShowStr(180, 80, "C", RED, BLACK, 16, 0);

        // 4. Tạo chuỗi gửi sang ESP
        char buffer[32];
        sprintf(buffer, "!TEMP:%.2f#", temperature);

        // 5. Gửi sang ESP bằng hàm của bạn
        uart_EspSendBytes((uint8_t*)buffer, strlen(buffer));
    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
