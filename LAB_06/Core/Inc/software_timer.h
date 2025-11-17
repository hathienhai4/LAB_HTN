/*
 * software_timer.h
 *
 *  Created on: Nov 3, 2025
 *      Author: Hai
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#include "main.h"

extern uint16_t flag_timer2;
extern uint16_t timer3_flag;
extern uint16_t timer4_flag;

void timer_init();
void setTimer2(uint16_t duration);
void setTimer3(uint16_t duration);
void setTimer4(uint16_t duration);

#endif /* INC_SOFTWARE_TIMER_H_ */
