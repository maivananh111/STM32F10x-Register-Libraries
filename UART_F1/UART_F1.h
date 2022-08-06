/*
 * UART_F1.h
 *
 *  Created on: Apr 15, 2022
 *      Author: A315-56
 */

#ifndef UART_F1_H_
#define UART_F1_H_

#include "stdio.h"


#ifdef __cplusplus
 extern "C" {
#endif

	void UART_Init(uint32_t Baudrate);
	void UART_Transmit(uint8_t Data);
	void UART_SendStr(char *String);
	uint8_t UART_Receive(void);
	void USART3_IRQHandler(void);
	 void UART_IT_RXCplt_CallBack(void);

#ifdef __cplusplus
 }
#endif

#endif /* UART_F1_H_ */
