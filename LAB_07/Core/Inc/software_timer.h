/*
 * software_timer.h
 *
 *  Created on: Nov 3, 2025
 *      Author: Hai
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#include "main.h"
#include "led_7seg.h"

extern uint16_t flag_timer2;
extern uint16_t flag_timer3;

void timer_init();
void setTimer2(uint16_t duration);
void setTimer3(uint16_t duration);
void timer_EnableDelayUs();
void delay_us (uint16_t us);

#endif /* INC_SOFTWARE_TIMER_H_ */
