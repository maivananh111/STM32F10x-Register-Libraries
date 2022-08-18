/*
 * SPI_F1.hpp
 *
 *  Created on: 12 thg 8, 2022
 *      Author: A315-56
*/

#pragma once

#include "DMA_F1.h"
#include "GPIO_F1.h"

#include "MVA_DEF.h"
#include "STM_LOG.h"
#include "stm32f1xx.h"


#define SPI_LOG_DEBUG_ENABLE

#define SPI_TIMEOUT 100UL

typedef enum{
	SPI_FULLDUPLEXMASTER = 0UL,
	SPI_HALFDUPLEXMASTER,
} SPI_Mode;

typedef enum{
	SPI_NORMAL_DMA = 0UL,
	SPI_INTERRUPT,
} SPI_Type;

typedef enum{
	SPI_INTR_RX = SPI_CR2_RXNEIE,
	SPI_INTR_TX = SPI_CR2_TXEIE,
	SPI_INTR_TX_RX = (SPI_CR2_RXNEIE | SPI_CR2_TXEIE),
} SPI_InterruptSelect;

typedef enum{
	SPI_DATA8BIT = 0x00UL,
	SPI_DATA16BIT = SPI_CR1_DFF,
} SPI_DataSize;

typedef enum{
	SPI_DATAMSB = 0x00UL,
	SPI_DATALSB = SPI_CR1_LSBFIRST,
} SPI_DataFormat;

typedef enum{
	SPI_CLOCKDIV2 = 0UL,
	SPI_CLOCKDIV4,
	SPI_CLOCKDIV8,
	SPI_CLOCKDIV16,
	SPI_CLOCKDIV32,
	SPI_CLOCKDIV64,
	SPI_CLOCKDIV128,
	SPI_CLOCKDIV256,
} SPI_ClockDiv;

typedef enum{
	SPI_CPOL0_CPHA0 = 0UL,
	SPI_CPOL0_CPHA1 = SPI_CR1_CPHA,
	SPI_CPOL1_CPHA0 = SPI_CR1_CPOL,
	SPI_CPOL1_CPHA1 = (SPI_CR1_CPOL | SPI_CR1_CPHA),
} SPI_ClockSample;

typedef struct {
	SPI_Mode SPIMode            = SPI_FULLDUPLEXMASTER;
	SPI_Type SPIType            = SPI_NORMAL_DMA;
	SPI_InterruptSelect InterruptSelect = SPI_INTR_RX;
	SPI_DataSize DataSize       = SPI_DATA8BIT;
	SPI_DataFormat DataFormat   = SPI_DATAMSB;
	SPI_ClockDiv ClockDiv       = SPI_CLOCKDIV4;
	SPI_ClockSample ClockSample = SPI_CPOL0_CPHA0;
	uint32_t InterruptPriority  = 0;
	DMA *TxDma = NULL;
	DMA *RxDma = NULL;
	GPIO_TypeDef *Port;
	uint16_t CLKPin;
	uint16_t MISOPin;
	uint16_t MOSIPin;
	bool PeriphRemap = false;
} SPI_Config;


template <typename DataSize>
class SPI {
	public:
		SPI(SPI_TypeDef *Spi);
		void Init(SPI_Config SpiConf);

		Result_t Transmit(DataSize *TxData, uint32_t Data_Number);
		Result_t Receive(DataSize *RxData, uint32_t Data_Number);
		Result_t Transmit_Receive(DataSize *TxData, DataSize *RxData, uint32_t Data_Number);

		Result_t Transmit(DataSize TxData);
		Result_t Receive(DataSize *RxData);
		Result_t Transmit_Receive(DataSize TxData, DataSize *RxData);

		Result_t Transmit_DMA(DataSize *TxData, uint32_t Data_Number);
		Result_t Receive_DMA(DataSize *RxData, uint32_t Data_Number);
		Result_t Transmit_Receive_DMA(DataSize *TxData, DataSize *RxData, uint32_t Data_Number);

		Result_t Stop_DMA(void);

		DMA *_TxDma, *_RxDma;
	private:
		SPI_TypeDef *_spi;
		SPI_Mode _mode = SPI_FULLDUPLEXMASTER;
		SPI_DataSize _size = SPI_DATA8BIT;
		IRQn_Type _IRQn = SPI1_IRQn;

};


template <typename DataSize>
SPI<DataSize>::SPI(SPI_TypeDef *Spi){
	_spi = Spi;
}

template <typename DataSize>
void SPI<DataSize>::Init(SPI_Config SpiConf){
	_mode = SpiConf.SPIMode;
	_size = SpiConf.DataSize;
	_TxDma = SpiConf.TxDma;
	_RxDma = SpiConf.RxDma;

	if     (_spi == SPI1) RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
	else if(_spi == SPI2) RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;

	if     (SpiConf.Port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(SpiConf.Port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(SpiConf.Port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIO_AFOutput(SpiConf.Port, SpiConf.CLKPin, GPIO_AF_PUSHPULL);
	GPIO_AFOutput(SpiConf.Port, SpiConf.MOSIPin, GPIO_AF_PUSHPULL);
	if(_mode == SPI_FULLDUPLEXMASTER) GPIO_Mode(SpiConf.Port, SpiConf.MISOPin, GPIO_INPUT_FLOATING);
	if(SpiConf.PeriphRemap && _spi == SPI1) GPIO_Remap(SPI1_Remap);

	_spi -> CR1 = 0x00UL;

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR1 &=~ SPI_CR1_RXONLY;
	_spi -> CR1 |= SpiConf.ClockSample | SPI_CR1_MSTR | (SpiConf.ClockDiv << SPI_CR1_BR_Pos) | SpiConf.DataFormat;
	_spi -> CR1 |= SpiConf.DataSize | SPI_CR1_SSM | SPI_CR1_SSI;
	if(_mode == SPI_HALFDUPLEXMASTER) _spi -> CR1 |= SPI_CR1_BIDIMODE;

	_spi -> CR2 = 0x00UL;
	if(SpiConf.SPIType == SPI_INTERRUPT) _spi -> CR2 = SpiConf.InterruptSelect;

	if(SpiConf.SPIType == SPI_INTERRUPT){
		if     (_spi == SPI1) _IRQn = SPI1_IRQn;
		else if(_spi == SPI2) _IRQn = SPI2_IRQn;
		__NVIC_SetPriority(_IRQn, SpiConf.InterruptPriority);
		__NVIC_EnableIRQ(_IRQn);
	}

	_spi -> CR1 |= SPI_CR1_SPE;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit(DataSize *TxData, uint32_t Data_Number){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t TxCount = Data_Number;

	if(_mode == SPI_HALFDUPLEXMASTER){
		_spi -> CR1 &=~ SPI_CR1_SPE;
		_spi -> CR1 |= SPI_CR1_BIDIOE;
	}

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	while(TxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE) {res.CodeLine = __LINE__;
	#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error timeout set TXE flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
	#endif
			return res;
		}

		_spi -> DR = (uint32_t)(*TxData);

		if(_size == SPI_DATA8BIT) TxData += sizeof(uint8_t);
		else if(_size == SPI_DATA16BIT) TxData += sizeof(uint16_t);
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error timeout reset BSY flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	if(_mode == SPI_FULLDUPLEXMASTER){
		__IO uint32_t clr_ovr = _spi -> DR;
		clr_ovr = _spi -> SR;
		(void)clr_ovr;
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive(DataSize *RxData, uint32_t Data_Number){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t RxCount = Data_Number;

	if(_mode == SPI_HALFDUPLEXMASTER) {
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
		return res;
	}

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	while(RxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE) {res.CodeLine = __LINE__;
	#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error timeout set TXE flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
	#endif
			return res;
		}
		_spi -> DR = 0x00UL;

		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE) {res.CodeLine = __LINE__;
	#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error timeout set RXNE flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
	#endif
			return res;
		}
		*RxData = (DataSize)_spi -> DR;

		if(_size == SPI_DATA8BIT) RxData += sizeof(uint8_t);
		else if(_size == SPI_DATA16BIT) RxData += sizeof(uint16_t);
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error timeout reset BSY flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	if(_mode == SPI_FULLDUPLEXMASTER){
		__IO uint32_t clr_ovr = _spi -> DR;
		clr_ovr = _spi -> SR;
		(void)clr_ovr;
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive(DataSize *TxData, DataSize *RxData, uint32_t Data_Number){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t TxRxCount = Data_Number;

	if(_mode == SPI_HALFDUPLEXMASTER) {
		res.Status = UNAVAILABLE;
		res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Unavailable receive function, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	while(TxRxCount--){
		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE) {res.CodeLine = __LINE__;
	#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error timeout set TXE flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
	#endif
			return res;
		}
		_spi -> DR = *TxData;

		res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(res.Status != OKE) {res.CodeLine = __LINE__;
	#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error timeout set RXNE flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
	#endif
			return res;
		}
		*RxData = (DataSize)_spi -> DR;

		if(_size == SPI_DATA8BIT) {
			TxData += sizeof(uint8_t);
			RxData += sizeof(uint8_t);
		}
		else if(_size == SPI_DATA16BIT) {
			TxData += sizeof(uint16_t);
			RxData += sizeof(uint16_t);
		}
	}

	res = WaitFlagTimeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error timeout reset BSY flag, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	if(_mode == SPI_FULLDUPLEXMASTER){
		__IO uint32_t clr_ovr = _spi -> DR;
		clr_ovr = _spi -> SR;
		(void)clr_ovr;
	}
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit(DataSize TxData){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	res = Transmit(&TxData, 1);
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive(DataSize *RxData){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	res = Receive(RxData, 1);
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive(DataSize TxData, DataSize *RxData){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	res = Transmit_Receive(&TxData, RxData, 1);
	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_DMA(DataSize *TxData, uint32_t Data_Number){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	_spi -> CR1 &=~ SPI_CR1_SPE;
	if(_mode == SPI_HALFDUPLEXMASTER) _spi -> CR1 |= SPI_CR1_BIDIOE;

	_spi -> CR2 &=~ SPI_CR2_TXDMAEN;

	res = _TxDma -> Start((uint32_t)TxData, (uint32_t) &(_spi -> DR), Data_Number);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error start TxDma, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_TXDMAEN;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Receive_DMA(DataSize *RxData, uint32_t Data_Number){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ SPI_CR2_RXDMAEN;

	res = _RxDma -> Start((uint32_t) &(_spi -> DR), (uint32_t)RxData, Data_Number);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error start RxDma, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_RXDMAEN;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Transmit_Receive_DMA(DataSize *TxData, DataSize *RxData, uint32_t Data_Number){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	res = _RxDma -> Start((uint32_t) &(_spi -> DR), (uint32_t)RxData, Data_Number);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error start RxDma, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	res = _TxDma -> Start((uint32_t)TxData, (uint32_t) &(_spi -> DR), Data_Number);
	if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
		STM_LOG(BOLD_RED, "SPI", "Error start TxDma, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		return res;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

	return res;
}

template <typename DataSize>
Result_t SPI<DataSize>::Stop_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	if(_spi -> CR2 & SPI_CR2_RXDMAEN) {
		res = _RxDma -> Stop();
		if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error stop RxDma, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		}
	}

	if(_spi -> CR2 & SPI_CR2_TXDMAEN) {
		res = _TxDma -> Stop();
		if(res.Status != OKE) {res.CodeLine = __LINE__;
#ifdef SPI_LOG_DEBUG_ENABLE
			STM_LOG(BOLD_RED, "SPI", "Error stop TxDma, Status: %d --- Error line: %d.", res.Status, res.CodeLine);
#endif
		}
	}
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	return res;
}













