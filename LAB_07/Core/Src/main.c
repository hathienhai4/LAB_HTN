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
#include "software_timer.h"
#include "led_7seg.h"
#include "button.h"
#include "lcd.h"
#include "picture.h"
#include "ds3231.h"
#include "uart.h"
#include "sensor.h"
#include "buzzer.h"
#include "touch.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Game States
#define GAME_MENU 0
#define GAME_PLAYING 1
#define GAME_OVER 2

// Screen dimensions (LCD ILI9341: 240x320)
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define GAME_AREA_TOP 70
#define GAME_AREA_BOTTOM 310
#define GAME_AREA_HEIGHT (GAME_AREA_BOTTOM - GAME_AREA_TOP)

// Snake settings
#define SNAKE_BLOCK_SIZE 6
#define MAX_SNAKE_LENGTH 200
#define INITIAL_SNAKE_LENGTH 3

// Grid size
#define GRID_WIDTH (SCREEN_WIDTH / SNAKE_BLOCK_SIZE)
#define GRID_HEIGHT (GAME_AREA_HEIGHT / SNAKE_BLOCK_SIZE)

// Directions
#define DIR_UP 0
#define DIR_DOWN 1
#define DIR_LEFT 2
#define DIR_RIGHT 3

// Button positions for menu
#define BTN_START_X1 60
#define BTN_START_Y1 140
#define BTN_START_X2 180
#define BTN_START_Y2 180

// Control buttons positions (top area)
#define BTN_UP_X 105
#define BTN_UP_Y 5
#define BTN_DOWN_X 105
#define BTN_DOWN_Y 40
#define BTN_LEFT_X 75
#define BTN_LEFT_Y 22
#define BTN_RIGHT_X 135
#define BTN_RIGHT_Y 22

#define BTN_SIZE 25

// Score position
#define SCORE_X 180
#define SCORE_Y 45

// Game state
typedef struct {
    uint8_t x;
    uint8_t y;
} Point;

typedef struct {
    Point body[MAX_SNAKE_LENGTH];
    uint16_t length;
    uint8_t direction;
    uint8_t next_direction;
} Snake;

typedef struct {
    Point position;
    uint8_t active;
} Food;

typedef struct {
    Snake snake;
    Food food;
    uint16_t score;
    uint8_t state;
    uint8_t game_speed;
} Game;

Game game;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FSMC_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void system_init();
void test_LedDebug();

// Snake game functions
void game_init();
void game_reset();
void game_draw_menu();
void game_draw_controls();
void game_draw_score();
void game_update();
void game_draw();
void snake_move();
uint8_t snake_check_collision();
void food_generate();
void food_draw();
uint8_t is_button_pressed(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void handle_touch_input();

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  system_init();
  game_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  touch_Adjust();
  lcd_Clear(BLACK);
  game_draw_menu();
  
  while (1)
  {
	  // Scan touch screen
	  touch_Scan();
	  
	  // Handle touch input
	  handle_touch_input();
	  
	  // 50ms task
	  if(flag_timer2 == 1){
		  flag_timer2 = 0;
		  test_LedDebug();
	  }
	  
	  // Game update (150ms for game speed)
	  if(flag_timer3 == 1){
		  flag_timer3 = 0;
		  if(game.state == GAME_PLAYING){
			  game_update();
			  game_draw();
		  }
	  }
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, FSMC_RES_Pin|T_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD_LATCH_Pin|T_CS_Pin|T_CLK_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : FSMC_RES_Pin T_MOSI_Pin */
  GPIO_InitStruct.Pin = FSMC_RES_Pin|T_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : LD_LATCH_Pin T_CS_Pin T_CLK_Pin */
  GPIO_InitStruct.Pin = LD_LATCH_Pin|T_CS_Pin|T_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : T_PEN_Pin T_MISO_Pin */
  GPIO_InitStruct.Pin = T_PEN_Pin|T_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
	timer_init();
	button_init();
	lcd_init();
	touch_init();
	setTimer2(50);
	setTimer3(150); // Game update timer
}

uint8_t count_led_debug = 0;

void test_LedDebug(){
	count_led_debug = (count_led_debug + 1)%20;
	if(count_led_debug == 0){
		HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
	}
}

// ==================== SNAKE GAME IMPLEMENTATION ====================

void game_init(){
	game.state = GAME_MENU;
	game.score = 0;
	game.game_speed = 150;
}

void game_reset(){
	// Initialize snake in the center
	game.snake.length = INITIAL_SNAKE_LENGTH;
	game.snake.direction = DIR_RIGHT;
	game.snake.next_direction = DIR_RIGHT;
	
	uint8_t start_x = GRID_WIDTH / 2;
	uint8_t start_y = GRID_HEIGHT / 2;
	
	for(int i = 0; i < INITIAL_SNAKE_LENGTH; i++){
		game.snake.body[i].x = start_x - i;
		game.snake.body[i].y = start_y;
	}
	
	game.score = 0;
	game.food.active = 0;
	food_generate();
}

void game_draw_menu(){
	lcd_Clear(BLACK);
	
	// Title
	lcd_ShowStr(50, 80, "SNAKE GAME", WHITE, BLACK, 24, 1);
	
	// Start button
	lcd_Fill(BTN_START_X1, BTN_START_Y1, BTN_START_X2, BTN_START_Y2, GREEN);
	lcd_ShowStr(BTN_START_X1 + 25, BTN_START_Y1 + 12, "START", BLACK, GREEN, 16, 1);
	
	// Instructions
	lcd_ShowStr(15, 220, "Touch controls to play", WHITE, BLACK, 16, 1);
}

void game_draw_controls(){
	// Draw control buttons with borders
	// UP button
	lcd_DrawRectangle(BTN_UP_X, BTN_UP_Y, BTN_UP_X + BTN_SIZE, BTN_UP_Y + BTN_SIZE, CYAN);
	lcd_ShowStr(BTN_UP_X + 5, BTN_UP_Y + 5, "U", WHITE, BLACK, 16, 1);
	
	// DOWN button
	lcd_DrawRectangle(BTN_DOWN_X, BTN_DOWN_Y, BTN_DOWN_X + BTN_SIZE, BTN_DOWN_Y + BTN_SIZE, CYAN);
	lcd_ShowStr(BTN_DOWN_X + 5, BTN_DOWN_Y + 5, "D", WHITE, BLACK, 16, 1);
	
	// LEFT button
	lcd_DrawRectangle(BTN_LEFT_X, BTN_LEFT_Y, BTN_LEFT_X + BTN_SIZE, BTN_LEFT_Y + BTN_SIZE, CYAN);
	lcd_ShowStr(BTN_LEFT_X + 5, BTN_LEFT_Y + 5, "L", WHITE, BLACK, 16, 1);
	
	// RIGHT button
	lcd_DrawRectangle(BTN_RIGHT_X, BTN_RIGHT_Y, BTN_RIGHT_X + BTN_SIZE, BTN_RIGHT_Y + BTN_SIZE, CYAN);
	lcd_ShowStr(BTN_RIGHT_X + 5, BTN_RIGHT_Y + 5, "R", WHITE, BLACK, 16, 1);
}

void game_draw_score(){
	char score_str[20];
	sprintf(score_str, "S:%d", game.score);
	lcd_Fill(SCORE_X, SCORE_Y, 235, SCORE_Y + 16, BLACK);
	lcd_ShowStr(SCORE_X, SCORE_Y, score_str, YELLOW, BLACK, 16, 1);
}

void food_generate(){
	if(game.food.active) return;
	
	// Generate random position
	uint8_t valid = 0;
	while(!valid){
		game.food.position.x = rand() % GRID_WIDTH;
		game.food.position.y = rand() % GRID_HEIGHT;
		
		// Check if food overlaps with snake
		valid = 1;
		for(int i = 0; i < game.snake.length; i++){
			if(game.snake.body[i].x == game.food.position.x && 
			   game.snake.body[i].y == game.food.position.y){
				valid = 0;
				break;
			}
		}
	}
	
	game.food.active = 1;
}

void food_draw(){
	if(game.food.active){
		uint16_t x = game.food.position.x * SNAKE_BLOCK_SIZE;
		uint16_t y = game.food.position.y * SNAKE_BLOCK_SIZE + GAME_AREA_TOP;
		lcd_Fill(x, y, x + SNAKE_BLOCK_SIZE - 1, y + SNAKE_BLOCK_SIZE - 1, RED);
	}
}

void snake_move(){
	// Update direction
	game.snake.direction = game.snake.next_direction;
	
	// Move body
	for(int i = game.snake.length - 1; i > 0; i--){
		game.snake.body[i] = game.snake.body[i-1];
	}
	
	// Move head
	switch(game.snake.direction){
		case DIR_UP:
			game.snake.body[0].y--;
			break;
		case DIR_DOWN:
			game.snake.body[0].y++;
			break;
		case DIR_LEFT:
			game.snake.body[0].x--;
			break;
		case DIR_RIGHT:
			game.snake.body[0].x++;
			break;
	}
}

uint8_t snake_check_collision(){
	Point head = game.snake.body[0];
	
	// Check wall collision
	if(head.x >= GRID_WIDTH || head.y >= GRID_HEIGHT){
		return 1;
	}
	
	// Check self collision
	for(int i = 1; i < game.snake.length; i++){
		if(game.snake.body[i].x == head.x && game.snake.body[i].y == head.y){
			return 1;
		}
	}
	
	return 0;
}

void game_update(){
	snake_move();
	
	// Check collision
	if(snake_check_collision()){
		game.state = GAME_OVER;
		lcd_Fill(40, 130, 200, 160, RED);
		lcd_ShowStr(55, 137, "GAME OVER!", WHITE, RED, 16, 1);
		char final_score[30];
		sprintf(final_score, "Score: %d", game.score);
		lcd_ShowStr(65, 180, final_score, YELLOW, BLACK, 16, 1);
		lcd_ShowStr(30, 220, "Touch to restart", WHITE, BLACK, 16, 1);
		return;
	}
	
	// Check food collision
	Point head = game.snake.body[0];
	if(game.food.active && head.x == game.food.position.x && head.y == game.food.position.y){
		// Eat food
		game.score += 10;
		game.food.active = 0;
		
		// Grow snake
		if(game.snake.length < MAX_SNAKE_LENGTH){
			game.snake.body[game.snake.length] = game.snake.body[game.snake.length - 1];
			game.snake.length++;
		}
		
		food_generate();
		game_draw_score();
	}
}

void game_draw(){
	// Clear game area (keep controls visible)
	lcd_Fill(0, GAME_AREA_TOP, SCREEN_WIDTH, GAME_AREA_BOTTOM, BLACK);
	
	// Draw snake
	for(int i = 0; i < game.snake.length; i++){
		uint16_t x = game.snake.body[i].x * SNAKE_BLOCK_SIZE;
		uint16_t y = game.snake.body[i].y * SNAKE_BLOCK_SIZE + GAME_AREA_TOP;
		uint16_t color = (i == 0) ? YELLOW : GREEN; // Head is yellow, body is green
		lcd_Fill(x, y, x + SNAKE_BLOCK_SIZE - 1, y + SNAKE_BLOCK_SIZE - 1, color);
	}
	
	// Draw food
	food_draw();
}

uint8_t is_button_pressed(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
	if(!touch_IsTouched()) return 0;
	uint16_t x = touch_GetX();
	uint16_t y = touch_GetY();
	return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
}

void handle_touch_input(){
	if(!touch_IsTouched()) return;
	
	switch(game.state){
		case GAME_MENU:
			// Check start button
			if(is_button_pressed(BTN_START_X1, BTN_START_Y1, BTN_START_X2, BTN_START_Y2)){
				game.state = GAME_PLAYING;
				game_reset();
				lcd_Clear(BLACK);
				game_draw_controls();
				game_draw_score();
				game_draw();
			}
			break;
			
		case GAME_PLAYING:
			// UP button
			if(is_button_pressed(BTN_UP_X, BTN_UP_Y, BTN_UP_X + BTN_SIZE, BTN_UP_Y + BTN_SIZE)){
				if(game.snake.direction != DIR_DOWN){
					game.snake.next_direction = DIR_UP;
				}
			}
			// DOWN button
			else if(is_button_pressed(BTN_DOWN_X, BTN_DOWN_Y, BTN_DOWN_X + BTN_SIZE, BTN_DOWN_Y + BTN_SIZE)){
				if(game.snake.direction != DIR_UP){
					game.snake.next_direction = DIR_DOWN;
				}
			}
			// LEFT button
			else if(is_button_pressed(BTN_LEFT_X, BTN_LEFT_Y, BTN_LEFT_X + BTN_SIZE, BTN_LEFT_Y + BTN_SIZE)){
				if(game.snake.direction != DIR_RIGHT){
					game.snake.next_direction = DIR_LEFT;
				}
			}
			// RIGHT button
			else if(is_button_pressed(BTN_RIGHT_X, BTN_RIGHT_Y, BTN_RIGHT_X + BTN_SIZE, BTN_RIGHT_Y + BTN_SIZE)){
				if(game.snake.direction != DIR_LEFT){
					game.snake.next_direction = DIR_RIGHT;
				}
			}
			break;
			
		case GAME_OVER:
			// Touch anywhere to restart
			HAL_Delay(500); // Debounce
			game.state = GAME_MENU;
			game_draw_menu();
			break;
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
