/*
 * uart.h
 *
 *  Created on: Nov 6, 2025
 *      Author: Hai
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include <stdio.h>
#include "utils.h"
#include "light_control.h"

#define BUFFER_SIZE 100

extern uint8_t flag_uart2;
extern uint8_t receive_buffer1;
extern uint8_t receive_buffer2;

void uart_init_rs232();

void uart_Rs232SendString(uint8_t* str);

void uart_Rs232SendBytes(uint8_t* bytes, uint16_t size);

void uart_Rs232SendNum(uint32_t num);

void uart_Rs232SendNumPercent(uint32_t num);

void uart_init_esp();
void uart_EspSendBytes(uint8_t* bytes, uint16_t size);

extern uint8_t buffer[BUFFER_SIZE];
extern volatile uint8_t uart_flag;
extern volatile uint8_t bf_head, bf_tail;

#endif /* INC_UART_H_ */
