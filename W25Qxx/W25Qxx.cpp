/*
 * W25Qxx.cpp
 *
 *  Created on: 2 thg 9, 2021
 *      Author: A315-56
 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "W25Qxx.h"
#include "GPIO_F1.h"
#include "System_Clock_F1.h"
#include "DMA_F1.h"


#define CMD_WRITE_DISABLE  0x04
#define CMD_WRITE_ENABLE   0x06
#define CMD_SECTOR_ERASE   0x20
#define CMD_BLOCK_ERASE    0xD8
#define CMD_CHIP_ERASE     0xC7
#define CMD_ENABLE_RESET   0x66
#define CMD_FAST_READ      0x0B
#define CMD_PAGE_PROGRAMM  0x02
#define CMD_GET_JEDEC_ID   0x9F
#define CMD_READ_STATUS_1  0x05
#define CMD_READ_STATUS_2  0x35
#define CMD_READ_STATUS_3  0x15
#define CMD_WRITE_STATUS   0x50
#define CMD_WRITE_STATUS_1 0x01
#define CMD_WRITE_STATUS_2 0x31
#define CMD_WRITE_STATUS_3 0x11

#define STATUS_REG_BUSY    0x01

w25qxx_t w25qxx;
SPI<uint8_t> *flash_spi;
DMA *flash_dmatx, *flash_dmarx;
volatile uint8_t  dma_flash_tx_flag = 0;
volatile uint8_t  dma_flash_rx_flag = 0;

W25Q::W25Q(GPIO_TypeDef *CS_Port, uint16_t CS_Pin){
    GPIOx = CS_Port;
    CS_pin = CS_Pin;
}

void W25Q::CS_Active(void) {
	GPIO_Reset(GPIOx, CS_pin);
}
void W25Q::CS_Idle(void) {
	GPIO_Set(GPIOx, CS_pin);
}

void W25Q::WriteEnable(void) {
	CS_Active();
	flash_spi -> Transmit(CMD_WRITE_ENABLE);
    CS_Idle();
}
void W25Q::WriteDisable(void) {
	CS_Active();
	flash_spi -> Transmit(CMD_WRITE_DISABLE);
    CS_Idle();
}

uint8_t W25Q::ReadStatusREG(uint8_t StatusRegisterNum) {
	uint8_t status = 0;
    CS_Active();
	switch(StatusRegisterNum){
		case 1:
			flash_spi -> Transmit(CMD_READ_STATUS_1);
			status = w25qxx.SR1 = flash_spi -> Transmit_Receive(0x00);
		break;
		case 2:
			flash_spi -> Transmit(CMD_READ_STATUS_2);
			status = w25qxx.SR2 = flash_spi -> Transmit_Receive(0x00);
		break;
		case 3:
			flash_spi -> Transmit(CMD_READ_STATUS_3);
			status = w25qxx.SR3 = flash_spi -> Transmit_Receive(0x00);
		break;
	}
	 CS_Idle();
     return status;
}
void W25Q::WriteStatusReg(uint8_t StatusRegisterNum, uint8_t cmd){
    WaitWriteEnd();
    WriteEnable();
    CS_Active();
    switch(StatusRegisterNum){
    	case 1:
    		flash_spi -> Transmit(CMD_WRITE_STATUS_1);
    	break;
    	case 2:
    		flash_spi -> Transmit(CMD_WRITE_STATUS_2);
    	break;
    	case 3:
    		flash_spi -> Transmit(CMD_WRITE_STATUS_3);
    	break;
    }
    flash_spi -> Transmit(cmd);
    CS_Idle();
    WriteDisable();
    WaitWriteEnd();
}

void W25Q::ProtectedChip(void){
	WriteStatusReg(1, 0xFF);
}
void W25Q::UnProtectedChip(void){
	WriteStatusReg(1, 0x00);
}

void W25Q::WaitWriteEnd(void) {
	CS_Active();
	flash_spi -> Transmit(CMD_READ_STATUS_1);
    do {
		w25qxx.SR1 = flash_spi -> Transmit_Receive(0x00);
		Tick_Delay_ms(1);
    } while ((w25qxx.SR1 & STATUS_REG_BUSY));
    CS_Idle();
}
void W25Q::SetAddress(uint32_t Address){
	if(w25qxx.Code >= 256){
		flash_spi -> Transmit((Address & 0xFF000000) >> 24);
	}
	flash_spi -> Transmit((Address & 0xFF0000) >> 16);
	flash_spi -> Transmit((Address & 0x00FF00) >> 8);
	flash_spi -> Transmit((Address & 0x0000FF)&0xFF);
}

uint32_t W25Q::ReadID(void) {
    uint32_t ID = 0;
    uint8_t RxBuf[3];
    CS_Idle();
    Tick_Delay_ms(100);
    CS_Active();
    flash_spi -> Transmit_Receive(CMD_GET_JEDEC_ID);
    flash_spi -> Receive(RxBuf, 3);
    CS_Idle();
    ID = (uint32_t)(RxBuf[0]<<16) | (uint32_t)(RxBuf[1]<<8) | (uint32_t)RxBuf[2];
    return ID;
}

uint16_t W25Q::Init(SPI<uint8_t> *spi, DMA *dmatx, DMA *dmarx){
	flash_spi = spi;
	flash_dmatx = dmatx;
	flash_dmarx = dmarx;
	GPIO_Mode(GPIOx, CS_pin, GPIO_Output_PushPull);

	flash_dmarx -> Init(DMA_Normal, DMA_PERIPH_TO_MEM, DMA_Data8Bit, DMA_Priority_VeryHigh);
	flash_dmatx -> Init(DMA_Normal, DMA_MEM_TO_PERIPH, DMA_Data8Bit, DMA_Priority_VeryHigh);
	flash_spi -> FullDuplexMaster_Init(SPI_CLKDiv4, SPI_Data8Bit, SPI_DataMSB, CPOL_0_CPHA_0, 0);

	uint32_t id = ReadID();
	switch(id & 0x000000FF){
		case 0x19:	// 	w25q256
			w25qxx.Code = 256;
			w25qxx.BlockCount = 512;
		break;
		case 0x18:	// 	w25q128
			w25qxx.Code = 128;
			w25qxx.BlockCount = 256;
		break;
		case 0x17:	//	w25q64
			w25qxx.Code = 64;
			w25qxx.BlockCount = 128;
		break;
		case 0x16:	//	w25q32
			w25qxx.Code = 32;
			w25qxx.BlockCount = 64;
		break;
		case 0x15:	//	w25q16
			w25qxx.Code = 16;
			w25qxx.BlockCount = 32;
		break;
		case 0x14:	//	w25q80
			w25qxx.Code = 80;
			w25qxx.BlockCount = 16;
		break;
		default:
			w25qxx.Code = 0;
		return 0;
	}
	w25qxx.PageSize = 256;
	w25qxx.SectorSize = 4096;
	w25qxx.SectorCount = w25qxx.BlockCount*16;
	w25qxx.PageCount = (w25qxx.SectorCount*w25qxx.SectorSize)/w25qxx.PageSize;
	w25qxx.BlockSize = w25qxx.SectorSize*16;
	w25qxx.Capacity = (w25qxx.SectorCount*w25qxx.SectorSize)/1024;
	ReadStatusREG(1);
	ReadStatusREG(2);
	ReadStatusREG(3);
	return id;
}

void W25Q::EraseSector(uint32_t SectorNum){
	uint32_t SectorAddr = SectorNum * w25qxx.SectorSize;
	WaitWriteEnd();
    WriteEnable();

    CS_Active();
    flash_spi -> Transmit(CMD_SECTOR_ERASE);
	SetAddress(SectorAddr);
	CS_Idle();

    WaitWriteEnd();
}
/*void W25Q::EraseBlock(uint32_t BlockAddr) {
	BlockAddr = BlockAddr * w25qxx.SectorSize * 16;
	WaitWriteEnd();
    WriteEnable();

    CS_Active();
    Transmit(CMD_BLOCK_ERASE);
    SetAddress(BlockAddr);
	CS_Idle();

	WaitWriteEnd();
}*/
void W25Q::EraseChip(void){
	WriteEnable();

	CS_Active();
	flash_spi -> Transmit(CMD_CHIP_ERASE);
    CS_Idle();

	WaitWriteEnd();
}

void W25Q::ReadBytes(uint32_t BytesAddress, uint8_t *Data, uint16_t NumByteRead){
	CS_Active();
	flash_spi -> Transmit(CMD_FAST_READ);
	SetAddress(BytesAddress);
	flash_spi -> Transmit(0);

	uint8_t *TxDummyBuf = (uint8_t *)malloc(NumByteRead * sizeof(uint8_t));
	memset(TxDummyBuf, 0x00, NumByteRead);
	flash_spi -> Transmit_Receive_DMA(*flash_dmatx, *flash_dmarx, TxDummyBuf, Data, (uint16_t)NumByteRead);
	while(!dma_flash_rx_flag);
	dma_flash_tx_flag = 0;
	dma_flash_rx_flag = 0;
	free(TxDummyBuf);
//	Receive(Data, NumByteRead);
	CS_Idle();
}

void W25Q::WriteBytes(uint32_t WriteAddr, uint8_t *Data, uint16_t NumByteWrite) {
    WaitWriteEnd();
    WriteEnable();

    CS_Active();
    flash_spi -> Transmit(CMD_PAGE_PROGRAMM);
    SetAddress(WriteAddr);
//    Transmit(Data, NumByteWrite);
    flash_spi -> Transmit_DMA(*flash_dmatx, Data, (uint16_t)NumByteWrite);
	while(!dma_flash_tx_flag);
	dma_flash_tx_flag = 0;
	CS_Idle();

	WriteDisable();
    WaitWriteEnd();
}

uint32_t W25Q::ReadPage(uint32_t PageCount, uint8_t *Data, uint32_t OffsetByte, uint16_t NumByteRead) {
	if((NumByteRead > w25qxx.PageSize)||(NumByteRead == 0)) NumByteRead = w25qxx.PageSize;
	if((OffsetByte + NumByteRead) > w25qxx.PageSize) NumByteRead = w25qxx.PageSize - OffsetByte;
	uint32_t PageAddress = PageCount * w25qxx.PageSize;
	PageAddress = PageAddress + OffsetByte;
	ReadBytes(PageAddress, Data, NumByteRead);
	return PageAddress;
}

uint32_t W25Q::WritePage(uint32_t PageCount, uint8_t *Data, uint32_t OffsetByte, uint16_t NumByteWrite) {
	if((NumByteWrite > w25qxx.PageSize)||(NumByteWrite == 0)) NumByteWrite = w25qxx.PageSize;
	if((OffsetByte + NumByteWrite) > w25qxx.PageSize) NumByteWrite = w25qxx.PageSize - OffsetByte;
	uint32_t PageAddress = PageCount * w25qxx.PageSize;
	WriteBytes(PageAddress, Data, NumByteWrite);
	return PageAddress;
}

