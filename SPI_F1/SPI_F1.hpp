/*
 * SPI_F1.hpp
 *
 *  Created on: Jun 7, 2022
 *      Author: A315-56
 */

#pragma once

#include "stdio.h"
#include "stm32f1xx.h"
#include "stm32f103xb.h"

#include "DMA_F1.h"
#include "GPIO_F1.h"


typedef enum{
	SPI_Normal_or_DMA = 0,
	SPI_INTR_TX = 0x80UL,
	SPI_INTR_RX = 0x40UL,
	SPI_INTR_TX_RX = 0xC0UL
} SPI_Type;

typedef enum{
	SPI_Data8Bit = 0UL,
	SPI_Data16Bit = 0x0800UL
} SPI_DataSize;

typedef enum{
	SPI_DataMSB = 0UL,
	SPI_DataLSB = 0x80UL
} SPI_DataFormat;

typedef enum{
	SPI_CLKDiv2 = 0,
	SPI_CLKDiv4,
	SPI_CLKDiv8,
	SPI_CLKDiv16,
	SPI_CLKDiv32,
	SPI_CLKDiv64,
	SPI_CLKDiv128,
	SPI_CLKDiv256
} SPI_CLKDiv;

typedef enum{
	CPOL_0_CPHA_0 = 0,
	CPOL_0_CPHA_1,
	CPOL_1_CPHA_0,
	CPOL_1_CPHA_1,
} SPI_CLKMode;

template <typename DataSize>
class SPI{
	public:
		SPI(SPI_TypeDef *SPI, SPI_Type TYPE, GPIO_TypeDef *Port, uint16_t MOSIPin, uint16_t MISOPin, uint16_t CLKPin);
		SPI_TypeDef *FullDuplexMaster_Init(SPI_CLKDiv DIV, SPI_DataSize SIZE, SPI_DataFormat FORMAT, SPI_CLKMode CLKMODE, uint8_t INTR_Priority = 0);
		SPI_TypeDef *HalfDuplexMaster_Init(SPI_CLKDiv DIV, SPI_DataSize SIZE, SPI_DataFormat FORMAT, SPI_CLKMode CLKMODE, uint8_t INTR_Priority = 0);
		void Transmit(DataSize DATA);
		void Transmit(DataSize *DATA, uint32_t Size);
		DataSize Receive(void);
		void Receive(DataSize *DATA, uint32_t Size);
		DataSize Transmit_Receive(DataSize DATA);
		void Transmit_Receive(DataSize *TxDATA, DataSize *RxDATA, uint16_t Size);
		void Transmit_DMA(DMA SPI_DMA, DataSize *DATA, uint16_t Size);
		void Receive_DMA (DMA SPI_DMA, DataSize *DATA, uint16_t Size);
		void Transmit_Receive_DMA(DMA Tx_Dma, DMA Rx_Dma, DataSize *TxData, DataSize *RxData, uint16_t Size);
		void Stop_DMA(DMA dma);

	private:
		uint8_t spi_num = 1;
		SPI_TypeDef *_spi;
		SPI_Type _type;
		SPI_CLKDiv _div;
		SPI_DataSize _size;
		SPI_DataFormat _format;
		SPI_CLKMode _clkmode;

		GPIO_TypeDef *_port;
		uint16_t _miso, _mosi, _clk;
};

template <typename DataSize>
SPI<DataSize>::SPI(SPI_TypeDef *SPI, SPI_Type TYPE, GPIO_TypeDef *Port, uint16_t MOSIPin, uint16_t MISOPin, uint16_t CLKPin){
	_spi  = SPI;
	_type = TYPE;
	_port = Port;
	_miso = MISOPin;
	_mosi = MOSIPin;
	_clk  = CLKPin;
}

template <typename DataSize>
SPI_TypeDef *SPI<DataSize>::FullDuplexMaster_Init(SPI_CLKDiv DIV, SPI_DataSize SIZE, SPI_DataFormat FORMAT, SPI_CLKMode CLKMODE, uint8_t INTR_Priority){
	_div = DIV;
	_size = SIZE;
	_format = FORMAT;
	_clkmode = CLKMODE;

	/* ENABLE SPI CLOCK */
	if     (_spi == SPI1) RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
	else if(_spi == SPI2) RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;

	if     (_port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(_port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(_port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIO_AFOutput(_port, _clk, GPIO_AF_PushPull);
	GPIO_AFOutput(_port, _mosi, GPIO_AF_PushPull);
	GPIO_Mode(_port, _miso, GPIO_Input_Floating);

	/* ENABLE SPI FULL DUPLEX MASTER, SOFTWARE SS */
	_spi -> CR1 &=~ SPI_CR1_RXONLY;
	_spi -> CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | _size | _format | (_div << SPI_CR1_BR_Pos) | _clkmode | SPI_CR1_SPE;
	_spi -> CR2 |= _type;
	/* ENABLE SPI INTERRUPT */
	if(_type > SPI_Normal_or_DMA){
		IRQn_Type IRQ;
		if     (_spi == SPI1) IRQ = SPI1_IRQn;
		else if(_spi == SPI2) IRQ = SPI2_IRQn;
		__NVIC_SetPriority(IRQ, INTR_Priority);
		__NVIC_EnableIRQ(IRQ);
	}
	return _spi;
}

template <typename DataSize>
SPI_TypeDef *SPI<DataSize>::HalfDuplexMaster_Init(SPI_CLKDiv DIV, SPI_DataSize SIZE, SPI_DataFormat FORMAT, SPI_CLKMode CLKMODE, uint8_t INTR_Priority){
	_div = DIV;
	_size = SIZE;
	_format = FORMAT;
	_clkmode = CLKMODE;
	/* ENABLE SPI CLOCK */
	if     (_spi == SPI1) RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
	else if(_spi == SPI2) RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;

	if     (_port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(_port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(_port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIO_AFOutput(_port, _clk, GPIO_AF_PushPull);
	GPIO_AFOutput(_port, _mosi, GPIO_AF_PushPull);

	/* ENABLE SPI HALF DUPLEX MASTER 1 LINE MODE, SOFTWARE SS */
	_spi -> CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | _size | _format | (_div << SPI_CR1_BR_Pos) | _clkmode | SPI_CR1_SPE;
	_spi -> CR1 &=~ SPI_CR1_RXONLY;
	_spi -> CR2 |= _type;
	/* ENABLE SPI INTERRUPT */
	if(_type > SPI_Normal_or_DMA){
		IRQn_Type IRQ;
		if     (_spi == SPI1) IRQ = SPI1_IRQn;
		else if(_spi == SPI2) IRQ = SPI2_IRQn;
		__NVIC_SetPriority(IRQ, INTR_Priority);
		__NVIC_EnableIRQ(IRQ);
	}
	return _spi;
}

template <typename DataSize>
void SPI<DataSize>::Transmit(DataSize *DATA, uint32_t Size){
	uint32_t i = 0;
	while(i < Size){
	   while (!((_spi -> SR) & SPI_SR_TXE));
	   _spi -> DR = DATA[i];
	   i++;
	}
	while (!((_spi -> SR) & SPI_SR_TXE));
	while (((_spi -> SR) & SPI_SR_BSY));
	DataSize temp = _spi -> DR;
	temp = _spi -> SR;
	(void)temp;
}

template <typename DataSize>
void SPI<DataSize>::Receive(DataSize *DATA, uint32_t Size){
	uint32_t i = Size;
	while(i){
		while (((_spi -> SR) & SPI_SR_BSY));
		_spi -> DR = 0x00;
		while (!((_spi -> SR) & SPI_SR_RXNE));
		*DATA++ = (_spi -> DR);
		i--;
	}
}

template <typename DataSize>
void SPI<DataSize>::Transmit(DataSize DATA){
	while (!((_spi -> SR) & SPI_SR_TXE));
	_spi -> DR = DATA;
	while (!((_spi -> SR) & SPI_SR_TXE));
	while (((_spi -> SR) & SPI_SR_BSY));
	DataSize temp = _spi -> DR;
	temp = _spi -> SR;
	(void)temp;
}

template <typename DataSize>
DataSize SPI<DataSize>::Receive(void){
	while(!(_spi -> SR & SPI_SR_RXNE));
	DataSize temp = _spi -> DR;
	return temp;
}

template <typename DataSize>
DataSize SPI<DataSize>::Transmit_Receive(DataSize DATA){
	while (((_spi -> SR) & SPI_SR_BSY));
	_spi -> DR = DATA;
	while (!((_spi -> SR) & SPI_SR_TXE));
	while (!((_spi -> SR) & SPI_SR_RXNE));
	DataSize temp = _spi -> DR;
	return temp;
}

template <typename DataSize>
void SPI<DataSize>::Transmit_Receive(DataSize *TxDATA, DataSize *RxDATA, uint16_t Size){
	uint32_t byte_writed = Size;
	while(byte_writed){
		/* WAIT FOR TX DATA IS EMPTY */
		while(!(_spi -> SR & SPI_SR_TXE));
		/* TRANSMIT DATA */
		_spi -> DR = *TxDATA++;
		*RxDATA++ = Receive();
		byte_writed--;
	}
	/* WAIT FOR TX DATA IS EMPTY */
	while(!(_spi -> SR & SPI_SR_TXE));
	/* WAIT FOR BSY BIT RESET */
	while((_spi -> SR & SPI_SR_BSY));
	/* READ DR AND SR REGISTER FOR CLEAR OVERRUN */
	__IO uint32_t ovr = _spi -> DR;
	ovr = _spi -> SR;
	(void)ovr;
}

template <typename DataSize>
void SPI<DataSize>::Transmit_DMA(DMA SPI_DMA, DataSize *DATA, uint16_t Size){
	/* DISABLE SPI */
	_spi -> CR1 &=~ SPI_CR1_SPE;
	/* DISABLE SPI TRANSMIT WITH DMA */
	_spi -> CR2 &=~ SPI_CR2_TXDMAEN;
	/* SETUP DMA */
	SPI_DMA.SetDirection(DMA_MEM_TO_PERIPH);
	SPI_DMA.Start((uint32_t)DATA, (uint32_t)&_spi -> DR, Size);
	/* ENABLE SPI TRANSMIT WITH DMA */
	_spi -> CR2 |= SPI_CR2_TXDMAEN;
	/* ENABLE SPI */
	_spi -> CR1 |= SPI_CR1_SPE;
}

template <typename DataSize>
void SPI<DataSize>::Receive_DMA (DMA SPI_DMA, DataSize *DATA, uint16_t Size){
	/* DISABLE SPI */
	_spi -> CR1 &=~ SPI_CR1_SPE;
	/* DISABLE SPI RECEIVE WITH DMA */
	_spi -> CR2 &=~ SPI_CR2_RXDMAEN;
	/* SETUP DMA */
	SPI_DMA.SetDirection(DMA_PERIPH_TO_MEM);
	SPI_DMA.Start((uint32_t)&_spi -> DR, (uint32_t)DATA, Size);
	/* ENABLE SPI RECEIVE WITH DMA */
	_spi -> CR2 |= SPI_CR2_RXDMAEN;
	/* ENABLE SPI */
	_spi -> CR1 |= SPI_CR1_SPE;
}

template <typename DataSize>
void SPI<DataSize>::Transmit_Receive_DMA(DMA Tx_Dma, DMA Rx_Dma, DataSize *TxData, DataSize *RxData, uint16_t Size){
	/* DISABLE SPI */
	_spi -> CR1 &=~ SPI_CR1_SPE;
	/* DISABLE SPI TRANSMIT AND RECEIVE WITH DMA */
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	/* SETUP SPI RECEIVE DMA */
	Rx_Dma.SetDirection(DMA_PERIPH_TO_MEM);
	Rx_Dma.Start((uint32_t)&_spi -> DR, (uint32_t)RxData, Size);
	/* SETUP SPI TRANSMIT DMA */
	Tx_Dma.SetDirection(DMA_MEM_TO_PERIPH);
	Tx_Dma.Start((uint32_t)TxData, (uint32_t)&_spi -> DR, Size);
	/* ENABLE SPI TRANSMIT AND RECEIVE WITH DMA */
	_spi -> CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
	/* ENABLE SPI */
	_spi -> CR1 |= SPI_CR1_SPE;
}

template <typename DataSize>
void SPI<DataSize>::Stop_DMA(DMA dma){
	dma.Stop();
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
}







