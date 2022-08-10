/*
 * USART_F1.cpp
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#include "USART_F1.h"
#include "stm32f1xx.h"
#include "GPIO_F1.h"
#include "SYSCLK_F1.h"
#include "math.h"


#define USART_TIMEOUT 100U

USART::USART(USART_TypeDef *usart){
	_usart = usart;
	_type  = USART_No_INTR;
}

void USART::Init(USART_Config usart_conf){
	_txdma = usart_conf.TxDma;
	_rxdma = usart_conf.RxDma;
	uint32_t USART_BusFreq = 0UL;

	if     (_usart == USART1) {
		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
		USART_BusFreq = GetBusFreq(PCLK2);
	}
	else if(_usart == USART2) {
		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
		USART_BusFreq = GetBusFreq(PCLK1);
	}
	else if(_usart == USART3) {
		RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
		USART_BusFreq = GetBusFreq(PCLK1);
	}

	if     (usart_conf.Port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(usart_conf.Port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(usart_conf.Port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIO_AFOutput(usart_conf.Port, usart_conf.TxPin, GPIO_AF_PushPull);
	GPIO_Mode(usart_conf.Port, usart_conf.RxPin, GPIO_Input_PullUp);
	GPIO_Pullup(usart_conf.Port, usart_conf.RxPin);

	_usart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	if(_type == USART_INTR || _type == USART_INTR) _usart -> CR1 |= USART_CR1_RXNEIE; // | USART_CR1_TCIE.

	float USARTDIV = (float)(USART_BusFreq/(usart_conf.Baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
//	if(((uint16_t)(Fraction * 100) % 100) == 0) DIV_Fraction = (uint16_t)(Fraction);
//	else DIV_Fraction = (uint16_t)(Fraction) + 1;
	DIV_Fraction = ceil(Fraction);
	_usart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	if(_type == USART_INTR || _type == USART_INTR_DMA){
		IRQn_Type IRQ;
		if(_usart == USART1) IRQ = USART1_IRQn;
		if(_usart == USART2) IRQ = USART2_IRQn;
		if(_usart == USART3) IRQ = USART3_IRQn;
		__NVIC_SetPriority(IRQ, 0);
		__NVIC_EnableIRQ(IRQ);
	}
	Transmit('\n');
}

Result_t USART::Transmit(uint8_t Data){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	_usart -> DR = Data;
	res = WaitFlagTimeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
	if(res.Status != OKE)res.CodeLine = __LINE__;
	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return res;
}

Result_t USART::SendString(char *String){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	while(*String) {
		res = Transmit(*String++);
		if(res.Status != OKE){
			res.CodeLine = __LINE__;
			return res;
		}
	}
	return res;
}

Result_t USART::Receive(uint8_t *Data){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	res = WaitFlagTimeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	*Data = _usart -> DR;

	return res;
}

Result_t USART::TransmitDMA(uint8_t *TxData, uint16_t Length){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	_txdma -> SetDirection(DMA_MEM_TO_PERIPH);
	_usart -> CR3 &=~ USART_CR3_DMAT;
	res = _txdma -> Start((uint32_t)TxData, (uint32_t)&_usart -> DR, Length);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	_usart -> SR &=~ USART_SR_TC;
	_usart -> CR3 |= USART_CR3_DMAT;

	return res;
}

Result_t USART::ReceiveDMA(uint8_t *RxData, uint16_t Length){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	_rxdma -> SetDirection(DMA_PERIPH_TO_MEM);
	_usart -> CR3 &=~ USART_CR3_DMAR;
	res = _rxdma -> Start((uint32_t)&_usart -> DR, (uint32_t)RxData, Length);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;
	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;
	_usart -> CR3 |= USART_CR3_DMAR;

	return res;
}

Result_t USART::Stop_Transmit_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_usart -> CR3 & USART_CR3_DMAT){
		_usart -> CR3 &=~ USART_CR3_DMAT;
		res = _txdma -> Stop();
		if(res.Status != OKE){
			res.CodeLine = __LINE__;
			return res;
		}
		_usart -> CR1 &=~ USART_CR1_TXEIE;
	}
	else{
		res.CodeLine = __LINE__;
		res.Status = ERR;
	}

	return res;
}

Result_t USART::Stop_Receive_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_usart -> CR3 & USART_CR3_DMAR){
		_usart -> CR3 &=~ USART_CR3_DMAR;
		res = _rxdma -> Stop();
		if(res.Status != OKE){
			res.CodeLine = __LINE__;
			return res;
		}
		_usart -> CR1 &=~ USART_CR1_PEIE;
		_usart -> CR3 &=~ USART_CR3_EIE;
	}
	else{
		res.CodeLine = __LINE__;
		res.Status = ERR;
	}

	return res;
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







