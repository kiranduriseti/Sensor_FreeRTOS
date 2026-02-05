/*
 * UART_print.c
 *
 *  Created on: Dec 20, 2025
 *      Author: Kiran Duriseti
 */
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "gpio.h"
#include "stm32L4xx_hal.h"
#include "UART_print.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#define BufferSize 32
#define IO_SIZE 256

char inputs[IO_SIZE];
uint8_t input_size = 0;

uint8_t tx_buffer1[IO_SIZE];
uint8_t tx_buffer2[IO_SIZE];

uint8_t *active = tx_buffer1;
uint8_t *pending = tx_buffer2;
uint32_t pending_size = 0;

static volatile uint8_t tx_busy = 0;

#include <sys/unistd.h>  // for STDOUT_FILENO

void UART_print(char *msg){
	if (msg == NULL) return;

	taskENTER_CRITICAL();

	if (!tx_busy){
		sprintf((char*)active, "%s", msg);

		tx_busy = 1;

		taskEXIT_CRITICAL();

		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)active, strlen((const char *)active));

//		if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)active, strlen(active)) != HAL_OK){
//			tx_busy = 0;
//		}
	}
	else {
		sprintf((char*)pending, "%s", msg);

		pending_size = strlen(msg);


		taskEXIT_CRITICAL();
	}
}

void print_thread (char* msg) {
	if (msg == NULL) return;

	osMutexAcquire(uartMutexHandle, osWaitForever);
	UART_print(msg);
	osMutexRelease(uartMutexHandle);
}

int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        static char buf[IO_SIZE];

        int copy_len = (len < IO_SIZE - 1) ? len : IO_SIZE - 1;
        memcpy(buf, ptr, copy_len);
        buf[copy_len] = '\0';

        UART_print(buf);
        return len;
    }

    return -1;
}

void UART_print_blocking(char* msg){
	//DO NOT USE
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(1000);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance != USART2) return;

	UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();

	if (pending_size == 0) {
		tx_busy = 0;
		taskEXIT_CRITICAL_FROM_ISR(mask);
		return;
	}

	uint8_t *tmp = active;
	active = pending;
	pending = tmp;

	uint16_t len = pending_size;
	pending_size = 0;

	taskEXIT_CRITICAL_FROM_ISR(mask);

	//if (HAL_UART_GetState(huart) == HAL_UART_STATE_READY) {
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)active, len);
	//}
}
