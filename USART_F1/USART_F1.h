/*
 * USART_F1.h
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#ifndef USART_F1_H_
#define USART_F1_H_

#include "stdio.h"
#include "stm32f1xx.h"
#include "DMA_F1.h"

//#define USE_USART1_ISR
//#define USE_USART2_ISR
#define USE_USART3_ISR

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	USART_No_INTR = 0,
	USART_INTR,
	USART_DMA,
	USART_INTR_DMA
} USART_Type;

typedef enum{
	USART_CLKNoDiv = 1,
	USART_CLKDiv2  = 2,
	USART_CLKDiv4  = 4,
	USART_CLKDiv8  = 8,
	USART_CLKDiv16 = 16,
} USART_CLKDiv;

class USART {
	public:
		USART(USART_TypeDef *usart, USART_CLKDiv CLKDiv = USART_CLKNoDiv, USART_Type Type = USART_No_INTR);
		void Init(uint32_t Baudrate, GPIO_TypeDef *Port, uint16_t TxPin, uint16_t RxPin);
		void Transmit(uint8_t Data);
		void TransmitDMA(DMA dmatx, uint8_t *TxData, uint16_t Length);
		void SendString(char *String);
		uint8_t Receive(void);
		void ReceiveDMA(DMA dmarx, uint8_t *RxData, uint16_t Length);
		void TransmitStopDMA(DMA dmatx);
		void ReceiveStopDMA(DMA dmarx);

	private:
		USART_TypeDef *_usart;
		USART_CLKDiv _div;
		USART_Type _type;
//		uint8_t _usart_num = 1;
};

#ifdef USE_USART1_ISR
void USART1_IRQHandler(void);
void USART1_RXCplt_CallBack(void);
void USART1_TXCplt_CallBack(void);
#endif
#ifdef USE_USART2_ISR
void USART2_IRQHandler(void);
void USART2_RXCplt_CallBack(void);
void USART2_TXCplt_CallBack(void);
#endif
#ifdef USE_USART3_ISR
void USART3_IRQHandler(void);
void USART3_RXCplt_CallBack(void);
void USART3_TXCplt_CallBack(void);
#endif

#ifdef __cplusplus
}
#endif



#endif /* USART_F1_H_ */
