/*
 * W25Qxx.h
 *
 *  Created on: 2 thg 9, 2021
 *      Author: A315-56
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

#pragma once

#include "stdio.h"
#include "stm32f1xx.h"
#include "SPI_F1.hpp"


#ifdef __cplusplus
extern "C"{
#endif

typedef struct {
	uint16_t Code;
	uint16_t PageSize;
	uint32_t PageCount;
	uint32_t SectorSize;
	uint32_t SectorCount;
	uint32_t BlockSize;
	uint32_t BlockCount;
	uint32_t Capacity;
	uint8_t	 SR1;
	uint8_t	 SR2;
	uint8_t	 SR3;
}w25qxx_t;
extern w25qxx_t w25qxx;

extern DMA *flash_dmatx, *flash_dmarx;
extern SPI<uint8_t> *flash_spi;
extern volatile uint8_t  dma_flash_tx_flag;
extern volatile uint8_t  dma_flash_rx_flag;

class W25Q{
	public:
		W25Q(GPIO_TypeDef *CS_Port, uint16_t CS_Pin);
		uint32_t ReadID(void);
		uint16_t Init(SPI<uint8_t> *spi, DMA *dmatx, DMA *dmarx);

		void EraseSector(uint32_t SectorNum);
//		void EraseBlock(uint32_t BlockAddr);
		void EraseChip(void);

		void ProtectedChip(void);
		void UnProtectedChip(void);

//		void ReadByte (uint32_t BytesAddress, uint8_t *Data);
		void ReadBytes(uint32_t BytesAddress, uint8_t *Data, uint16_t NumByteRead = 256);
		uint32_t ReadPage (uint32_t PageCount, uint8_t *Data, uint32_t OffsetByte = 0, uint16_t NumByteRead = 256);
//		void ReadSector(uint32_t SectorAddress, uint8_t *Data, uint32_t OffsetByte = 0, uint16_t NumByteRead = 4096);
//		void ReadBlock(uint32_t BlockAddress, uint8_t *Data, uint32_t OffsetByte = 0, uint16_t NumByteRead = 65536);

//		void WriteByte(uint32_t WriteAddr, uint8_t Data);
		void WriteBytes(uint32_t WriteAddr, uint8_t *Data, uint16_t NumByteWrite = 256);
		uint32_t WritePage(uint32_t PageCount, uint8_t *Data, uint32_t OffsetByte = 0, uint16_t NumByteWrite = 256);
//		void WriteSector(uint32_t SectorAddress, uint8_t *Data, uint32_t OffsetByte = 0,uint16_t NumByteWrite = 4096);
//		void WriteBlock(uint32_t BlockAddress, uint8_t *Data, uint32_t OffsetByte = 0, uint16_t NumByteWrite = 65536);

	private:
		GPIO_TypeDef *GPIOx;
		uint16_t CS_pin;

		void CS_Active(void);
		void CS_Idle(void);
		void WriteEnable(void);
		void WriteDisable(void);
		void WaitWriteEnd(void);
		uint8_t  ReadStatusREG(uint8_t StatusRegisterNum);
		void WriteStatusReg(uint8_t StatusRegisterNum, uint8_t cmd);
		void SetAddress(uint32_t Address);
		uint32_t BlockToPage(uint32_t	BlockAddress);
		uint32_t SectorToPage(uint32_t SectorAddress);
};


#ifdef __cplusplus
}
#endif

#endif /* INC_W25QXX_H_ */
