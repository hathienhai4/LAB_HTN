/*
 * software_timer.c
 *
 *  Created on: Nov 3, 2025
 *      Author: Hai
 */

#include "software_timer.h"
#include "led_7seg.h"

#define TIMER_CYCLE_2 1

//software timer variable
uint16_t flag_timer2 = 0;
uint16_t timer2_counter = 0;
uint16_t timer2_MUL = 0;

uint16_t timer3_flag = 0;
uint16_t timer3_counter = 0;
uint16_t timer3_MUL = 0;

uint16_t timer4_flag = 0;
uint16_t timer4_counter = 0;
uint16_t timer4_MUL = 0;


/**
  * @brief  Init timer interrupt
  * @param  None
  * @retval None
  */
void timer_init(){
	HAL_TIM_Base_Start_IT(&htim2);
}


/**
  * @brief  Set duration of software timer interrupt
  * @param  duration Duration of software timer interrupt
  * @retval None
  */
void setTimer2(uint16_t duration){
	timer2_MUL = duration/TIMER_CYCLE_2;
	timer2_counter = timer2_MUL;
	flag_timer2 = 0;
}

void setTimer3(uint16_t duration){
	timer3_MUL = duration/TIMER_CYCLE_2;
	timer3_counter = timer3_MUL;
	timer3_flag = 0;
}

void setTimer4(uint16_t duration){
	timer4_MUL = duration/TIMER_CYCLE_2;
	timer4_counter = timer4_MUL;
	timer4_flag = 0;
}

/**
  * @brief  Timer interrupt routine
  * @param  htim TIM Base handle
  * @note	This callback function is called by system
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		if(timer2_counter > 0){
			timer2_counter--;
			if(timer2_counter == 0) {
				flag_timer2 = 1;
				timer2_counter = timer2_MUL;
			}
		}
		
		if(timer3_counter > 0){
			timer3_counter--;
			if(timer3_counter == 0) {
				timer3_flag = 1;
				timer3_counter = timer3_MUL;
			}
		}
		
		if(timer4_counter > 0){
			timer4_counter--;
			if(timer4_counter == 0) {
				timer4_flag = 1;
				timer4_counter = timer4_MUL;
			}
		}
		// 1ms interrupt here
//		led7_Scan();
	}
}
