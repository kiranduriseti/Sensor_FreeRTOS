/*
 * UART_print.h
 *
 *  Created on: Jan 6, 2026
 *      Author: Kiran Duriseti
 */

#ifndef INC_UART_PRINT_H_
#define INC_UART_PRINT_H_

#include "cmsis_os.h"   // needed for osMutexId_t

extern osMutexId_t uartMutexHandle;
extern volatile int g_stop_req;
void uart_start_rx_it();
void UART_print(char *msg);
void UART_print_blocking(char *msg);
void print_thread (char* msg);

#endif /* INC_UART_PRINT_H_ */
