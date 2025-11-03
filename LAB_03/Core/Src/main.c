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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INIT		1
#define RED_GREEN	2
#define RED_YELLOW	3
#define GREEN_RED	4
#define YELLOW_RED	5
#define MOD_RED		6
#define MOD_GREEN	7
#define MOD_YELLOW	8

#define MAN_RED		16
#define MAN_GREEN	17
#define MAN_YELLOW	18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void system_init();
void test_LedDebug();
void test_LedY0();
void test_LedY1();
void test_7seg();
void test_button();
void test_lcd();
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
  /* USER CODE BEGIN 2 */
  system_init();
  lcd_Clear(WHITE);
  test_lcd();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while(!flag_timer2);
	  flag_timer2 = 0;
	  led7_Scan();
	  button_Scan();
//	  test_button();
	  fsm_traffic_light_run();
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DEBUG_LED_Pin|OUTPUT_Y0_Pin|OUTPUT_Y1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSMC_RES_GPIO_Port, FSMC_RES_Pin, GPIO_PIN_RESET);

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
	  setTimer2(50);
}

uint8_t count_led_debug = 0;
uint8_t count_led_Y0 = 0;
uint8_t count_led_Y1 = 0;

void test_LedDebug(){
	count_led_debug = (count_led_debug + 1)%40;
	if(count_led_debug == 0){
		HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
	}
}

void test_LedY0(){
	count_led_Y0 = (count_led_Y0+ 1)%120;
	if(count_led_Y0 > 40){
		HAL_GPIO_WritePin(OUTPUT_Y0_GPIO_Port, OUTPUT_Y0_Pin, 1);
	} else {
		HAL_GPIO_WritePin(OUTPUT_Y0_GPIO_Port, OUTPUT_Y0_Pin, 0);
	}
}

void test_LedY1(){
	count_led_Y1 = (count_led_Y1+ 1)%120;
	if(count_led_Y1 > 20){
		HAL_GPIO_WritePin(OUTPUT_Y1_GPIO_Port, OUTPUT_Y1_Pin, 1);
	} else {
		HAL_GPIO_WritePin(OUTPUT_Y0_GPIO_Port, OUTPUT_Y1_Pin, 0);
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
			lcd_ShowIntNum(136, 125, i, 2, BRED, GREEN, 32);
		}
	}
}

uint8_t cnt_2 = 0;
uint8_t cnt_3 = 0;
uint8_t cnt_4 = 0;

void test_lcd(){
	lcd_Fill(0, 0, 240, 20, BLUE);
	lcd_StrCenter(0, 2, "Hello World !!!", RED, BLUE, 16, 1);
	lcd_ShowStr(55, 30, "- Mode 1 -", WHITE, RED, 24, 0);
	lcd_ShowStr(50, 60, "Normal Mode", WHITE, RED, 24, 0);
//	lcd_DrawCircle(60, 140, RED, 40, 1);
	lcd_ShowStr(32, 190, "DEN 1", BLUE, WHITE, 24, 0);
//	lcd_DrawCircle(160, 140, GREEN, 40, 1);
	lcd_ShowStr(130, 190, "DEN 2", BLUE, WHITE, 24, 0);
	lcd_ShowPicture(80, 220, 90, 90, gImage_logo);
}

void lcd_mod2() {
	lcd_Fill(0, 0, 240, 20, BLUE);
	lcd_StrCenter(0, 2, "Hello World !!!", RED, BLUE, 16, 1);
	lcd_ShowStr(55, 30, "- Mode 2 -", WHITE, RED, 24, 0);
	lcd_ShowStr(50, 60, "Modify RED", WHITE, RED, 24, 0);
	lcd_ShowPicture(80, 220, 90, 90, gImage_logo);
	if (cnt_2 == 0)
		lcd_DrawCircle(60, 140, RED, 40, 1);
	else if (cnt_2 == 10)
		lcd_DrawCircle(60, 140, WHITE, 40, 1);
	cnt_2 = (cnt_2 + 1) % 20;
}

void lcd_mod3() {
	lcd_Fill(0, 0, 240, 20, BLUE);
	lcd_StrCenter(0, 2, "Hello World !!!", RED, BLUE, 16, 1);
	lcd_ShowStr(55, 30, "- Mode 3 -", WHITE, GREEN, 24, 0);
	lcd_ShowStr(50, 60, "Modify GREEN", WHITE, GREEN, 24, 0);
	lcd_ShowPicture(80, 220, 90, 90, gImage_logo);
	if (cnt_3 == 0)
		lcd_DrawCircle(60, 140, GREEN, 40, 1);
	else if (cnt_3 == 10)
		lcd_DrawCircle(60, 140, WHITE, 40, 1);
	cnt_3 = (cnt_3 + 1) % 20;
}

void lcd_mod4() {
	lcd_Fill(0, 0, 240, 20, BLUE);
	lcd_StrCenter(0, 2, "Hello World !!!", RED, BLUE, 16, 1);
	lcd_ShowStr(55, 30, "- Mode 4 -", WHITE, YELLOW, 24, 0);
	lcd_ShowStr(50, 60, "Modify YELLOW", WHITE, YELLOW, 24, 0);
	lcd_DrawCircle(60, 140, YELLOW, 40, 1);
	lcd_ShowPicture(80, 220, 90, 90, gImage_logo);
	if (cnt_4 == 0)
		lcd_DrawCircle(60, 140, GREEN, 40, 1);
	else if (cnt_4 == 10)
		lcd_DrawCircle(60, 140, WHITE, 40, 1);
	cnt_4 = (cnt_4 + 1) % 20;
}

int status = INIT;

int count_RG = 0;
int count_RY = 0;

int tmp_R = 0;
int tmp_Y = 0;
int tmp_G = 0;


int value = 0;

int TIME_RED = 5;
int TIME_YELLOW = 1;
int TIME_GREEN = 3;

int TIME_RG = 0;

int TIME_RED_tmp = 0;
int TIME_YELLOW_tmp = 0;
int TIME_GREEN_tmp = 0;


void fsm_traffic_light_run() {
	switch (status) {
	case INIT:
		status = RED_GREEN;
		count_RG = TIME_RED * 20;
		TIME_RG = (TIME_RED - TIME_GREEN) * 20;
		lcd_Clear(WHITE);
		break;
	case RED_GREEN:
		test_lcd();
		lcd_DrawCircle(60, 140, RED, 40, 1);
		lcd_DrawCircle(160, 140, GREEN, 40, 1);
		tmp_R = count_RG / 20;
		tmp_G = (count_RG - TIME_RG) / 20;

		if(button_count[12] == 1) {
			status = MOD_RED;
			value = TIME_RED;
			lcd_Clear(WHITE);
		}

		if (count_RG > 40) {
			lcd_ShowIntNum(38, 125, tmp_R, 2, WHITE, RED, 32);
			lcd_ShowIntNum(136, 125, tmp_G, 2, WHITE, GREEN, 32);
		}
		else {
			status = RED_YELLOW;
			count_RY = TIME_RG;
		}
		--count_RG;
		break;
	case RED_YELLOW:
		test_lcd();
		lcd_DrawCircle(60, 140, RED, 40, 1);
		lcd_DrawCircle(160, 140, YELLOW, 40, 1);
		tmp_R = count_RY / 20;
		tmp_Y = count_RY / 20;

		if(button_count[12] == 1) {
			status = MOD_RED;
			value = TIME_RED;
			lcd_Clear(WHITE);
		}

		if (count_RY > 0) {
			lcd_ShowIntNum(38, 125, tmp_R, 2, WHITE, RED, 32);
			lcd_ShowIntNum(136, 125, tmp_Y, 2, WHITE, YELLOW, 32);
		}
		else {
			status = GREEN_RED;
			count_RG = TIME_RED * 20;
		}
		--count_RY;
		break;
	case GREEN_RED:
		test_lcd();
		lcd_DrawCircle(60, 140, GREEN, 40, 1);
		lcd_DrawCircle(160, 140, RED, 40, 1);
		tmp_R = count_RG / 20;
		tmp_G = (count_RG - TIME_RG) / 20;

		if(button_count[12] == 1) {
			status = MOD_RED;
			value = TIME_RED;
			lcd_Clear(WHITE);
		}

		if (count_RG > 40) {
			lcd_ShowIntNum(38, 125, tmp_G, 2, WHITE, GREEN, 32);
			lcd_ShowIntNum(136, 125, tmp_R, 2, WHITE, RED, 32);
		}
		else {
			status = YELLOW_RED;
			count_RY = TIME_RG;
		}
		--count_RG;
		break;
	case YELLOW_RED:
		test_lcd();
		lcd_DrawCircle(60, 140, YELLOW, 40, 1);
		lcd_DrawCircle(160, 140, RED, 40, 1);
		tmp_R = count_RY / 20;
		tmp_Y = count_RY / 20;

		if(button_count[12] == 1) {
			status = MOD_RED;
			value = TIME_RED;
//			setTimer2(500);
			lcd_Clear(WHITE);
		}

		if (count_RY > 0) {
			lcd_ShowIntNum(38, 125, tmp_Y, 2, WHITE, YELLOW, 32);
			lcd_ShowIntNum(136, 125, tmp_R, 2, WHITE, RED, 32);
		}
		else {
			status = RED_GREEN;
			count_RG = TIME_RED * 20;
		}
		--count_RY;
		break;
	case MOD_RED:
		lcd_mod2();
		if (button_count[13] == 1) {
			value = (value >= 99) ? 1 : value + 1;
		}
		else if (button_count[14] == 1) {
			TIME_RED_tmp = value;
		}
		else if (button_count[12] == 1) {
			status = MOD_GREEN;
			value = TIME_GREEN;
			lcd_Clear(WHITE);
		}
		lcd_ShowIntNum(136, 125, value, 2, WHITE, RED, 32);
		break;
	case MOD_GREEN:
		lcd_mod3();
		if (button_count[13] == 1) {
			value = (value >= 99) ? 1 : value + 1;
		}
		else if (button_count[14] == 1) {
			TIME_GREEN_tmp = value;
		}
		else if (button_count[12] == 1) {
			status = MOD_YELLOW;
			value = TIME_YELLOW;
			lcd_Clear(WHITE);
		}
		lcd_ShowIntNum(136, 125, value, 2, WHITE, GREEN, 32);
		break;
	case MOD_YELLOW:
		lcd_mod4();
		if (button_count[13] == 1) {
			value = (value >= 99) ? 1 : value + 1;
		}
		else if (button_count[14] == 1) {
			TIME_YELLOW_tmp = value;
			if (TIME_RED_tmp == 0 && TIME_YELLOW_tmp == 0 && TIME_GREEN_tmp == 0)
				break;
			if (TIME_GREEN_tmp + TIME_YELLOW_tmp == TIME_RED_tmp) {
				TIME_GREEN = TIME_GREEN_tmp;
				TIME_YELLOW = TIME_YELLOW_tmp;
				TIME_RED = TIME_RED_tmp;
			}
		}
		else if (button_count[12] == 1) {
			status = RED_GREEN;
			count_RG = TIME_RED * 20;
			TIME_RG = (TIME_RED - TIME_GREEN) * 20;
			lcd_Clear(WHITE);
		}
		lcd_ShowIntNum(136, 125, value, 2, WHITE, YELLOW, 32);
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
