/*
 * uart.c
 *
 *  Created on: Nov 17, 2025
 *      Author: Hai
 */

#include "uart.h"

uint8_t receive_buffer1 = 0;
uint8_t msg[BUFFER_SIZE];
uint8_t buffer[BUFFER_SIZE];
volatile uint8_t uart_flag = 0;
volatile uint8_t bf_head = 0, bf_tail = 0;
void uart_init_rs232(){
	HAL_UART_Receive_IT(&huart1, &receive_buffer1, 1);
}

void uart_Rs232SendString(uint8_t* str){
	HAL_UART_Transmit(&huart1, (void*)msg, sprintf((void*)msg,"%s",str), 10);
}

void uart_Rs232SendBytes(uint8_t* bytes, uint16_t size){
	HAL_UART_Transmit(&huart1, bytes, size, 10);
}

void uart_Rs232SendNum(uint32_t num){
	if(num == 0){
		uart_Rs232SendString("0");
		return;
	}
    uint8_t num_flag = 0;
    int i;
	if(num < 0) uart_Rs232SendString("-");
    for(i = 10; i > 0; i--)
    {
        if((num / mypow(10, i-1)) != 0)
        {
            num_flag = 1;
            sprintf((void*)msg,"%d",num/mypow(10, i-1));
            uart_Rs232SendString(msg);
        }
        else
        {
            if(num_flag != 0)
            	uart_Rs232SendString("0");
        }
        num %= mypow(10, i-1);
    }
}

void uart_Rs232SendNumPercent(uint32_t num)
{
	sprintf((void*)msg,"%ld",num/100);
    uart_Rs232SendString(msg);
    uart_Rs232SendString(".");
    sprintf((void*)msg,"%ld",num%100);
    uart_Rs232SendString(msg);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		// rs232 isr
//		HAL_UART_Transmit(&huart1, &receive_buffer1, 1, 10);
		uint8_t next = (bf_head + 1) % BUFFER_SIZE;
		if(next != bf_tail){
			buffer[bf_head] = receive_buffer1;
			bf_head = next;
		}
		else {
			char* err_msg = "Buffer is full\n";
			HAL_UART_Transmit(&huart1,err_msg, strlen(err_msg), 10);
			buffer[bf_head] = receive_buffer1;
			bf_head = next;
//			bf_tail = (bf_tail + 1) % BUFFER_SIZE;
		}
		uart_flag = 1;
		// turn on the receice interrupt
		HAL_UART_Receive_IT(&huart1, &receive_buffer1, 1);
	}
}
