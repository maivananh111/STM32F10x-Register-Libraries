/*
 * I2C_F1.h
 *
 *  Created on: Aug 4, 2022
 *      Author: A315-56
 */

#ifndef I2C_F1_H_
#define I2C_F1_H_


#include "stdio.h"
#include "stm32f1xx.h"
#include "DMA_F1.h"
#include "MVA_DEF.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	I2C_StandardMode = 0,
	I2C_FastMode,
} I2C_Mode;

typedef enum{
	I2C_Duty_2 = 0,
	I2C_Duty_16_9,
} I2C_SCLDuty;

typedef enum{
	I2C_Addr_7Bit = 0,
	I2C_Addr_10Bit,
} I2C_AddrMode;

typedef enum{
	I2C_Write = 0,
	I2C_Read,
} I2C_Action;

typedef struct{
	I2C_Mode Mode;
	uint32_t Frequency;
	I2C_AddrMode AddressMode = I2C_Addr_7Bit;
	I2C_SCLDuty SCLDuty = I2C_Duty_2;
	GPIO_TypeDef *Port;
	uint16_t SCLPin;
	uint16_t SDAPin;
	bool PeriphRemap = false;
	DMA *TxDma = NULL;
	DMA *RxDma = NULL;
	bool GeneralCall = false;
	bool DualAddrMode = false;
	uint32_t Address = 0;
	uint32_t Address_2 = 0;
} I2C_Config;

class I2C{
	public:
		I2C(I2C_TypeDef *I2C);
		Result_t Init(I2C_Config i2c_conf);

		Result_t WaitBusy(void);

		Result_t SendStart(void);
		Result_t SendRepeatStart(void);
		Result_t SendSlaveAddr(uint16_t Slave_Address, I2C_Action Action);
		Result_t SendStart_SlaveAddr(uint16_t Slave_Address, I2C_Action Action);
		Result_t SendRepeatStart_SlaveAddr(uint16_t Slave_Address, I2C_Action Action);

		Result_t Transmit(uint8_t *TxData, uint16_t Size);
		Result_t Transmit(uint8_t TxData);

		Result_t Receive(uint8_t *Data, uint16_t Size);
		Result_t Receive(uint8_t *Data);

		Result_t Transmit_DMA(uint8_t *TxData, uint16_t Size);
		Result_t Receive_DMA(uint8_t *RxData, uint16_t Size);

		Result_t Stop_Transmit_DMA(void);
		Result_t Stop_Receive_DMA(void);

		Result_t SendStop(void);

		Result_t Memory_Transmit(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size);
		Result_t Memory_Receive(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size);

		DMA *_txdma, *_rxdma;

	private:
		I2C_TypeDef *_i2c;
		I2C_AddrMode _addrmode = I2C_Addr_7Bit;
		void ACKError_Action(void);
		void ClearADDR(void);
};

#ifdef __cplusplus
}
#endif

#endif /* I2C_F1_H_ */
