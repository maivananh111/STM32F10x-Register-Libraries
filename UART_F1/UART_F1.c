/*
 * UART.c
 *
 *  Created on: Apr 15, 2022
 *      Author: A315-56
 */

#include "UART_F1.h"
#include "stm32f1xx.h"
#include "GPIO_F1.h"


void UART_Init(uint32_t Baudrate){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;

	GPIO_AFOutput(GPIOB, 10, GPIO_AF_PushPull);
	GPIO_Mode(GPIOB, 11, GPIO_Input_PullUp);
	GPIO_Pullup(GPIOB, 11);

	USART3 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

	float USARTDIV = (float)((SystemCoreClock/2.0)/(Baudrate * 16.0));

	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	if(((uint16_t)(Fraction * 100) % 100) == 0) DIV_Fraction = (uint16_t)(Fraction);
	else DIV_Fraction = (uint16_t)(Fraction) + 1;
	USART3 -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	__NVIC_SetPriority(USART3_IRQn, 0);
	__NVIC_EnableIRQ(USART3_IRQn);
	UART_Transmit('\n');
}

void UART_Transmit(uint8_t Data){
	USART3 -> DR = Data;
	while (!(USART3 -> SR & USART_SR_TC));
	__IO uint32_t ovr = USART3 -> DR;
	ovr = USART3 -> SR;
	(void)ovr;
}

uint8_t UART_Receive(void){
	uint8_t Read_Data;
	while (!(USART3 -> SR & USART_SR_RXNE));
	Read_Data = USART3 -> DR;
	return Read_Data;
}

void UART_SendStr(char *String){
	while(*String) UART_Transmit(*String++);
}

void USART3_IRQHandler(void){
	if(USART3 -> SR & USART_SR_RXNE){
		UART_IT_RXCplt_CallBack();
	}
}

//__WEAK void UART_IT_RXCplt_CallBack(void){
//
//}

