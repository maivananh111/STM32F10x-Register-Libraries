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
#include "MVA_DEF.h"
#include "STM_LOG.h"

#define SPI_TIMEOUT 100U

typedef enum{
	FullDuplexMaster = 0,
	HalfDuplexMaster = 1,
} SPI_Mode;

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

typedef struct{
	SPI_Mode SPIMode          = FullDuplexMaster;
	SPI_Type SPIType          = SPI_Normal_or_DMA;
	SPI_DataSize DataSize     = SPI_Data8Bit;
	SPI_DataFormat DataFormat = SPI_DataMSB;
	SPI_CLKDiv ClockDiv       = SPI_CLKDiv2;
	SPI_CLKMode ClockMode     = CPOL_0_CPHA_0;
	uint8_t INTRPriority      = 0;
	GPIO_TypeDef *Port;
	uint16_t MOSIPin;
	uint16_t MISOPin;
	uint16_t CLKPin;
	bool PeriphRemap = false;
	DMA *TxDma = NULL;
	DMA *RxDma = NULL;
} SPI_Config;

template <typename DataSize>
class SPI{
	public:
		SPI(SPI_TypeDef *SPI);

		SPI_TypeDef *Init(SPI_Config spi_conf);

		Result_t Transmit(DataSize Data);
		Result_t Transmit(DataSize *Data, uint32_t Size);

		Result_t Receive(DataSize *Data);
		Result_t Receive(DataSize *Data, uint32_t Size);

		Result_t Transmit_Receive(DataSize TxData, DataSize *RxData);
		Result_t Transmit_Receive(DataSize *TxData, DataSize *RxData, uint16_t Size);

		Result_t Transmit_DMA(DataSize *Data, uint16_t Size);
		Result_t Receive_DMA(DataSize *Data, uint16_t Size);

		Result_t Transmit_Receive_DMA(DataSize *TxData, DataSize *RxData, uint16_t Size);

		Result_t Stop_Transmit_DMA(void);
		Result_t Stop_Receive_DMA(void);

		DMA *_txdma, *_rxdma;
		SPI_TypeDef *_spi;

	private:
		uint8_t spi_num = 1;

		SPI_Mode _mode;
		SPI_Type _type;

};

template <typename DataSize>
SPI<DataSize>::SPI(SPI_TypeDef *SPI){
	_spi  = SPI;
	_mode = FullDuplexMaster;
	_type = SPI_Normal_or_DMA;
}

template <typename DataSize>
SPI_TypeDef *SPI<DataSize>::Init(SPI_Config spi_conf){
	_mode  = spi_conf.SPIMode;
	_type  = spi_conf.SPIType;
	_txdma = spi_conf.TxDma;
	_rxdma = spi_conf.RxDma;

	/* ENABLE SPI CLOCK */
	if     (_spi == SPI1) RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
	else if(_spi == SPI2) RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;

	if     (spi_conf.Port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(spi_conf.Port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(spi_conf.Port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIO_AFOutput(spi_conf.Port, spi_conf.CLKPin, GPIO_AF_PushPull);
	GPIO_AFOutput(spi_conf.Port, spi_conf.MOSIPin, GPIO_AF_PushPull);
	if(_mode == FullDuplexMaster) GPIO_Mode(spi_conf.Port, spi_conf.MISOPin, GPIO_Input_Floating);
	if(spi_conf.PeriphRemap && _spi == SPI1) GPIO_Remap(SPI1_Remap);

	/* ENABLE SPI MASTER, SOFTWARE SS */
	_spi -> CR1 &=~ SPI_CR1_RXONLY;
	_spi -> CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | spi_conf.DataSize | spi_conf.DataFormat | (spi_conf.ClockDiv << SPI_CR1_BR_Pos) | spi_conf.ClockMode | SPI_CR1_SPE;
	if(_mode == HalfDuplexMaster) _spi -> CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
	_spi -> CR2 |= spi_conf.SPIType;

	/* ENABLE SPI INTERRUPT */
	if(_type > SPI_Normal_or_DMA){
		IRQn_Type IRQ;
		if     (_spi == SPI1) IRQ = SPI1_IRQn;
		else if(_spi == SPI2) IRQ = SPI2_IRQn;
		__NVIC_SetPriority(IRQ, spi_conf.INTRPriority);
		__NVIC_EnableIRQ(IRQ);
	}
	return _spi;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit(DataSize *Data, uint32_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	uint32_t TxCount = Size;
	while(TxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	   _spi -> DR = *Data++;
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	DataSize temp = _spi -> DR | _spi -> SR;
	(void)temp;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive(DataSize *Data, uint32_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_mode == HalfDuplexMaster){
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	uint32_t RxCount = Size;
	while(RxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

		_spi -> DR = 0x00;

		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

		*Data++ = (uint8_t)(_spi -> DR);
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit(DataSize Data){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	_spi -> DR = Data;

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	DataSize temp = _spi -> DR | _spi -> SR;
	(void)temp;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive(DataSize *Data){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_mode == HalfDuplexMaster){
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	*Data = _spi -> DR;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive(DataSize TxData, DataSize *RxData){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_mode == HalfDuplexMaster){
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	_spi -> DR = TxData;

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	*RxData = (uint8_t)(_spi -> DR);

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive(DataSize *TxData, DataSize *RxData, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_mode == HalfDuplexMaster){
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	uint32_t TxRxCount = Size;
	while(TxRxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

		_spi -> DR = *TxData++;
		Receive(RxData++);
	}
	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	__IO uint32_t ovr = _spi -> DR | _spi -> SR;
	(void)ovr;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_DMA(DataSize *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ SPI_CR2_TXDMAEN;

	_txdma -> SetDirection(DMA_MEM_TO_PERIPH);
	res = _txdma -> Start((uint32_t)Data, (uint32_t)&_spi -> DR, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	_spi -> CR2 |= SPI_CR2_TXDMAEN;
	_spi -> CR1 |= SPI_CR1_SPE;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive_DMA(DataSize *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_mode == HalfDuplexMaster){
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ SPI_CR2_RXDMAEN;

	_rxdma -> SetDirection(DMA_PERIPH_TO_MEM);
	_rxdma -> Start((uint32_t)&_spi -> DR, (uint32_t)Data, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	_spi -> CR2 |= SPI_CR2_RXDMAEN;
	_spi -> CR1 |= SPI_CR1_SPE;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive_DMA(DataSize *TxData, DataSize *RxData, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_mode == HalfDuplexMaster){
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	_rxdma -> SetDirection(DMA_PERIPH_TO_MEM);
	res = _rxdma -> Start((uint32_t)&_spi -> DR, (uint32_t)RxData, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	_txdma -> SetDirection(DMA_MEM_TO_PERIPH);
	res = _txdma -> Start((uint32_t)TxData, (uint32_t)&_spi -> DR, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	_spi -> CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
	_spi -> CR1 |= SPI_CR1_SPE;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Stop_Transmit_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_spi -> CR2 & SPI_CR2_TXDMAEN){
		res = _txdma -> Stop();
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

		_spi -> CR2 &=~ SPI_CR2_TXDMAEN;
	}
	else{
		res.Status = ERR;
		res.CodeLine = __LINE__;
	}

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Stop_Receive_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_spi -> CR2 & SPI_CR2_RXDMAEN){
		res = _rxdma -> Stop();
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

		_spi -> CR2 &=~ SPI_CR2_RXDMAEN;
	}
	else{
		res.Status = ERR;
		res.CodeLine = __LINE__;
	}

	return res;
}







