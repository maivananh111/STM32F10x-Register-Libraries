/*
 * USART_F1.cpp
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#include "USART_F1.h"
#include "stm32f1xx.h"
#include "GPIO_F1.h"



USART::USART(USART_TypeDef *usart, USART_CLKDiv CLKDiv, USART_Type Type){
	_usart = usart;
	_type = Type;
	_div = CLKDiv;
}

void USART::Init(uint32_t Baudrate, GPIO_TypeDef *Port, uint16_t TxPin, uint16_t RxPin){
	if     (_usart == USART1) RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
	else if(_usart == USART2) RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
	else if(_usart == USART3) RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;

	if     (Port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(Port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(Port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIO_AFOutput(Port, TxPin, GPIO_AF_PushPull);
	GPIO_Mode(Port, RxPin, GPIO_Input_PullUp);
	GPIO_Pullup(Port, RxPin);

	_usart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	if(_type == USART_INTR || _type == USART_INTR) _usart -> CR1 |= USART_CR1_RXNEIE; // | USART_CR1_TCIE.

	float USARTDIV = (float)((SystemCoreClock/(float)_div)/(Baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	if(((uint16_t)(Fraction * 100) % 100) == 0) DIV_Fraction = (uint16_t)(Fraction);
	else DIV_Fraction = (uint16_t)(Fraction) + 1;
	_usart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	if(_type == USART_INTR || _type == USART_INTR_DMA){
		IRQn_Type IRQ;
		if(_usart == USART1) IRQ = USART1_IRQn;
		if(_usart == USART2) IRQ = USART2_IRQn;
		if(_usart == USART3) IRQ = USART3_IRQn;
		__NVIC_SetPriority(IRQ, 0);
		__NVIC_EnableIRQ(IRQ);
	}
}

void USART::Transmit(uint8_t Data){
	_usart -> DR = Data;
	while (!(_usart -> SR & USART_SR_TC));
	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;
}

void USART::SendString(char *String){
	while(*String) Transmit(*String++);
}

void USART::TransmitDMA(DMA dmatx, uint8_t *TxData, uint16_t Length){
	dmatx.SetDirection(DMA_MEM_TO_PERIPH);
	_usart -> CR3 &=~ USART_CR3_DMAT;
	dmatx.Start((uint32_t)TxData, (uint32_t)&_usart -> DR, Length);
	_usart -> SR &=~ USART_SR_TC;
	_usart -> CR3 |= USART_CR3_DMAT;
}

uint8_t USART::Receive(void){
	while (!(_usart -> SR & USART_SR_RXNE));
	return _usart -> DR;
}

void USART::ReceiveDMA(DMA dmarx, uint8_t *RxData, uint16_t Length){
	dmarx.SetDirection(DMA_PERIPH_TO_MEM);
	_usart -> CR3 &=~ USART_CR3_DMAR;
	dmarx.Start((uint32_t)&_usart -> DR, (uint32_t)RxData, Length);
	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;
	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;
	_usart -> CR3 |= USART_CR3_DMAR;
}

void USART::TransmitStopDMA(DMA dmatx){
	if(_usart -> CR3 & USART_CR3_DMAT){
		_usart -> CR3 &=~ USART_CR3_DMAT;
		dmatx.Stop();
//		_usart -> CR1 &=~ USART_CR1_TXEIE;
	}
}

void USART::ReceiveStopDMA(DMA dmarx){
	if(_usart -> CR3 & USART_CR3_DMAR){
		_usart -> CR3 &=~ USART_CR3_DMAR;
		dmarx.Stop();
		_usart -> CR1 &=~ USART_CR1_PEIE;
		_usart -> CR3 &=~ USART_CR3_EIE;
	}
}

#ifdef USE_USART1_ISR
void USART1_IRQHandler(void){
	if(USART1 -> SR & USART_SR_RXNE){
		USART1_RXCplt_CallBack();
	}
	if(USART1 -> SR & USART_SR_TXE){
		USART1_TXCplt_CallBack();
	}
}

__WEAK void USART1_RXCplt_CallBack(void){}
__WEAK void USART1_TXCplt_CallBack(void){}
#endif

#ifdef USE_USART2_ISR
void USART2_IRQHandler(void){
	if(USART2 -> SR & USART_SR_RXNE){
		USART2_RXCplt_CallBack();
	}
	if(USART2 -> SR & USART_SR_TXE){
		USART2_TXCplt_CallBack();
	}
}

__WEAK void USART2_RXCplt_CallBack(void){}
__WEAK void USART2_TXCplt_CallBack(void){}
#endif

#ifdef USE_USART3_ISR
void USART3_IRQHandler(void){
	if(USART3 -> SR & USART_SR_RXNE){
		USART3_RXCplt_CallBack();
	}
//	if(USART3 -> SR & USART_SR_TXE){
//		USART3_TXCplt_CallBack();
//	}
}
__WEAK void USART3_RXCplt_CallBack(void){}
__WEAK void USART3_TXCplt_CallBack(void){}
#endif







