/*
 * I2C_F1.cpp
 *
 *  Created on: Aug 4, 2022
 *      Author: A315-56
 */

#include "SYSCLK_F1.h"
#include "I2C_F1.h"
#include "GPIO_F1.h"

#define I2C_BUSY_TIMEOUT            1000U     // 25ms.
#define I2C_DEFAULT_TIMEOUT         1000U     // 100ms.
#define I2C_MIN_FREQ_STANDARD       2000000U // Minimum is 2MHz of APB1 bus in standard mode.
#define I2C_MIN_FREQ_FAST           4000000U // Minimum is 4MHz of APB1 bus in standard mode.

I2C::I2C(I2C_TypeDef *I2C){
	_i2c = I2C;
}

Result_t I2C::Init(I2C_Config i2c_conf){
	_TxDma = i2c_conf.TxDma;
	_RxDma = i2c_conf.RxDma;

	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(i2c_conf.Mode == I2C_STANDARD_MODE && i2c_conf.Frequency > 100000U) {
		res.Status = ERR;
		res.CodeLine = __LINE__;
		return res;
	}
	_addrmode = i2c_conf.AddressMode;

	/* ***********************I2C CLK ENABLE*********************** */
	if(_i2c == I2C1)      RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;
	else if(_i2c == I2C2) RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;

	/* ***********************GPIO CLK ENABLE********************** */
	if     (i2c_conf.Port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if(i2c_conf.Port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if(i2c_conf.Port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;

	/* ***********************GPIO INIT********************** */
	GPIO_AFOutput(i2c_conf.Port, i2c_conf.SCLPin, GPIO_AF_OPENDRAIN);
	GPIO_AFOutput(i2c_conf.Port, i2c_conf.SDAPin, GPIO_AF_OPENDRAIN);
	GPIO_Pullup(i2c_conf.Port, i2c_conf.SCLPin);
	GPIO_Pullup(i2c_conf.Port, i2c_conf.SDAPin);
	if(i2c_conf.PeriphRemap && _i2c == I2C1) GPIO_Remap(I2C1_Remap);

	/* ***********************DISABLE I2C********************** */
	_i2c -> CR1 &=~ I2C_CR1_PE;
	/* ***********************RESET I2C********************** */
	_i2c -> CR1 |= I2C_CR1_SWRST;
	Tick_Delay_ms(5);
	_i2c -> CR1 &= ~I2C_CR1_SWRST;
	/* ***********************CHECK MINIMUM ALLOWED FREQUENCY********************** */
	uint32_t apb1_freq = GetBusFreq(PCLK1);
	if(((i2c_conf.Frequency <= 100000U)? (apb1_freq < I2C_MIN_FREQ_STANDARD) : (apb1_freq < I2C_MIN_FREQ_FAST))) {
		res.Status = ERR;
		res.CodeLine = __LINE__;
		return res;
	}
	/* ***********************SET ABP1 FREQUENCY TO CR2 REGISTER********************** */
	_i2c -> CR2 |= (uint32_t)((apb1_freq / 1000000U) >> I2C_CR2_FREQ_Pos);
	/* ***********************SET SCL RISE TIME********************** */
	_i2c -> TRISE = (uint32_t)((i2c_conf.Mode == I2C_STANDARD_MODE)? ((apb1_freq/1000000U) + 1U) : ((((apb1_freq/1000000U) * 300U) / 1000U) + 1U));
	/* ***********************SET I2C SPEED********************** */
	if(i2c_conf.Mode == I2C_STANDARD_MODE){
		_i2c -> CCR &=~ I2C_CCR_FS;
		// T_high = T_low = CCR * T_PCLK1, T = 2 * T_high.
		uint32_t ccr_tmp = (uint32_t)((apb1_freq - 1U)/(2U * i2c_conf.Frequency) + 1U);
		if(ccr_tmp < 4U) ccr_tmp = 4U;
		_i2c -> CCR |= ccr_tmp;
	}
	else if(i2c_conf.Mode == I2C_FAST_MODE){
		_i2c -> CCR |= I2C_CCR_FS;
		if(i2c_conf.SCLDuty == I2C_DUTY_2){
			_i2c -> CCR &=~ I2C_CCR_DUTY;
			// T_high = (1/2)T_low = CCR * TPCLK1, T = 3 * T_high.
			uint32_t ccr_tmp = (uint32_t)((apb1_freq)/(3U * i2c_conf.Frequency));
			if(ccr_tmp < 1U) ccr_tmp = 1U;
			_i2c -> CCR |= ccr_tmp;

		}
		else if(i2c_conf.SCLDuty == I2C_DUTY_16_9){
			_i2c -> CCR |= I2C_CCR_DUTY;
			// T_high = (9/16)T_low = CCR * TPCLK1, T = 25 * T_high.
			uint32_t ccr_tmp = (uint32_t)((apb1_freq)/(25U * i2c_conf.Frequency));
			if(ccr_tmp < 4U) ccr_tmp = 4U;
			_i2c -> CCR |= ccr_tmp;
		}
	}
	/* ***********************GENERAL CALL********************** */
	if(i2c_conf.GeneralCall) _i2c -> CR1 |= I2C_CR1_ENGC;
	/* ***********************ADDRESS 7 OR 10BIT********************** */
	if(i2c_conf.AddressMode == I2C_ADDRESS_10BIT)     _i2c -> OAR1 |= (I2C_OAR1_ADDMODE | 0x00004000U) | i2c_conf.Address;
	else if(i2c_conf.AddressMode == I2C_ADDRESS_7BIT) _i2c -> OAR1 |= 0x00004000U | i2c_conf.Address;
	/* ***********************DUAL ADDRESS MODE********************** */
	if(i2c_conf.DualAddrMode) _i2c -> OAR2 |= I2C_OAR2_ENDUAL | i2c_conf.Address_2;
	/* ***********************ENABLE I2C PERIPHERAL********************** */
	_i2c -> CR1 |= I2C_CR1_PE;
	/* ***********************CLEAR ALL FLAG********************** */
	_i2c -> SR1 = 0UL;
	_i2c -> SR2 = 0UL;

	return res;
}

void I2C::ACKError_Action(void){
	_i2c -> CR1 |= I2C_CR1_STOP;
	_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
}

void I2C::ClearADDR(void){
	__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2;
	(void)tmp;
}

Result_t I2C::WaitBusy(void){
	Result_t res = {
		.Status = BUSY,
		.CodeLine = 0,
	};
	/* ***********************CHECK BUSY FLAG********************** */
	res = WaitFlagTimeout(&(_i2c -> SR2), I2C_SR2_BUSY, FLAG_RESET, I2C_BUSY_TIMEOUT);
	if(res.Status == OKE){
		res.Status = READY;
		res.CodeLine = __LINE__;
		return res;
	}
	return res;
}

Result_t I2C::SendStart(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	/* ***********************WAIT I2C NOT BUSY********************** */
	res = WaitBusy();
	if(res.Status != READY){ res.CodeLine = __LINE__; return res; }
	/* ***********************DISABLE POS********************** */
	_i2c -> CR1 &=~ I2C_CR1_POS;
	/* ***********************GENERATE START********************** */
	_i2c -> CR1 |= I2C_CR1_START;
	/* ***********************WAIT SB FLAG IS SET********************** */
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	return res;
}

Result_t I2C::SendRepeatStart(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	/* ***********************GENERATE START********************** */
	_i2c -> CR1 |= I2C_CR1_START;
	/* ***********************WAIT SB FLAG IS SET********************** */
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	return res;
}

Result_t I2C::SendSlaveAddr(uint16_t Slave_Address, I2C_Action Action){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	/* ***********************SEND SLAVE ADDRESS*********************** */
	if(_addrmode == I2C_ADDRESS_7BIT){
		if(Action == I2C_WRITE)_i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) & ~I2C_OAR1_ADD0); // SEND 8BIT SLAVE ADDRESS WITH LSB BIT IS RESET(TRANSMIT MODE).
		else                   _i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) |  I2C_OAR1_ADD0); // SEND 8BIT SLAVE ADDRESS WITH LSB BIT IS SET(RECEIVE MODE).
	}
	/* ***********************WAIT ADDR FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_ADDR, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR)
			ACKError_Action();
		res.CodeLine = __LINE__;
		return res;
	}
	/* ***********************CLEAR ADDR FLAG*********************** */
	ClearADDR();

	return res;
}

Result_t I2C::SendStart_SlaveAddr(uint16_t Slave_Address, I2C_Action Action){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	/* ************* SEND START CONDITION **************** */
	res.Status = SendStart().Status;
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	/* ************* SEND SLAVE ADDRESS **************** */
	res.Status = SendSlaveAddr(Slave_Address, Action).Status;
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	return res;
}

Result_t I2C::SendRepeatStart_SlaveAddr(uint16_t Slave_Address, I2C_Action Action){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	/* ************* SEND REPEAT START CONDITION **************** */
	res.Status = SendRepeatStart().Status;
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	/* ************* SEND SLAVE ADDRESS **************** */
	res.Status = SendSlaveAddr(Slave_Address, Action).Status;
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }

	return res;
}

Result_t I2C::Transmit(uint8_t *TxData, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t TxCount = Size;

	/* ***********************WAIT TXE FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR)
			ACKError_Action();
		res.CodeLine = __LINE__;
		return res;
	}
	/* ***********************TRANSMIT DATA*********************** */
	while(TxCount > 0U){
		/* ***********************WAIT TXE FLAG IS SET*********************** */
		res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
		if(res.Status != OKE){
			if(res.Status == ERR)
				ACKError_Action();
			res.CodeLine = __LINE__;
			return res;
		}
		/* ***********************WRITE DATA TO DR*********************** */
		_i2c -> DR = *TxData++;
		TxCount--;
		if((_i2c -> SR1 & I2C_SR1_BTF) && TxCount != 0U){
			/* ***********************WRITE DATA TO DR*********************** */
			_i2c -> DR = *TxData++;
			TxCount--;
		}
	}
	/* ***********************WAIT BTF FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR)
			ACKError_Action();
		res.CodeLine = __LINE__;
		return res;
	}

	return res;
}

Result_t I2C::Transmit(uint8_t TxData){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	/* ***********************WAIT TXE FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR)
			ACKError_Action();
		res.CodeLine = __LINE__;
		return res;
	}
	/* ***********************WRITE DATA TO DR*********************** */
	_i2c -> DR = TxData;
	/* ***********************WAIT BTF FLAG IS SET*********************** */
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR)
			ACKError_Action();
		res.CodeLine = __LINE__;
		return res;
	}

	return res;
}

Result_t I2C::Receive(uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t RxCount = Size;
	/* ********************START READ DATA****************** */
	// SETUP BEFORE READ DATA.
	if(RxCount == 0U){
		// CLEAR ADDR FLAG.
		ClearADDR();
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 1U){
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		ClearADDR();
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 2U){
		// ENABLE POS.
		_i2c -> CR1 |= I2C_CR1_POS;
		// CLEAR ADDR FLAG.
		ClearADDR();
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
	}
	else{
		// ENABLE ACK.
		_i2c -> CR1 |= I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		ClearADDR();
	}
	// START READ DATA.
	while(RxCount > 0U){
		if(RxCount <= 3U){
			if(RxCount == 1U){
				// WAIT RXNE IS SET.
				res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE){
					if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
					res.CodeLine = __LINE__;
					return res;
				}
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
			else if(RxCount == 2U){
				// WAIT BTF FLAG IS SET.
				res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE) { res.CodeLine = __LINE__; return res;}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;

			}
			else{
				// WAIT BTF FLAG IS SET.
				res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE) { res.CodeLine = __LINE__; return res;}
				// DISABLE ACK.
				_i2c -> CR1 &=~ I2C_CR1_ACK;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// WAIT BTF FLAG IS SET.
				res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE){
					res.CodeLine = __LINE__;
					return res;
				}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
		else{
			// WAIT RXNE IS SET.
			res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
			if(res.Status != OKE){
				if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
				res.CodeLine = __LINE__;
				return res;
			}
			// READ FORM DR.
			*Data++ = (uint8_t)_i2c -> DR;
			RxCount--;
			if(_i2c -> SR1 & I2C_SR1_BTF){
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
	}
	return res;
}

Result_t I2C::Receive(uint8_t *Data){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	// DISABLE ACK.
	_i2c -> CR1 &=~ I2C_CR1_ACK;
	// CLEAR ADDR FLAG.
	ClearADDR();
	// GENERATE STOP.
	_i2c -> CR1 |= I2C_CR1_STOP;
	// WAIT RXNE IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
		res.CodeLine = __LINE__;
		return res;
	}
	// READ FORM DR.
	*Data = (uint8_t)_i2c -> DR;

	return res;
}

Result_t I2C::SendStop(void){
	// SEND STOP CONDITION.
	_i2c -> CR1 |= I2C_CR1_STOP;

	return {OKE, 0 };
}

Result_t I2C::Transmit_DMA(uint8_t *TxData, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	// SETUP AND START DMA.
	res = _TxDma -> Start((uint32_t)TxData, (uint32_t)(&_i2c -> DR), Size);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	// ENABLE I2C TX DMA
	_i2c -> CR2 |= I2C_CR2_DMAEN;

	return res;
}

Result_t I2C::Receive_DMA(uint8_t *RxData, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	// SETUP AND START DMA.
	res = _RxDma -> Start((uint32_t)(&_i2c -> DR), (uint32_t)RxData, Size);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	// RECEIVE ONLY 1 BYTE, NO NEED ACK.
	if(Size == 1U) _i2c -> CR1 &=~ I2C_CR1_ACK;
	// IF RECEIVE MORE THAN 1 BYTE, ENABLE DMA CHECK LAST BYTE.
	else _i2c -> CR2 |= I2C_CR2_LAST;
	// ENABLE I2C RX DMA
	_i2c -> CR2 |= I2C_CR2_DMAEN;

	return res;
}

Result_t I2C::Stop_Transmit_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_i2c -> CR2 & I2C_CR2_DMAEN){
		_TxDma -> Stop();
		_i2c -> CR2 &=~ I2C_CR2_DMAEN;
	}
	else{
		res.Status = ERR;
		res.CodeLine = __LINE__;
	}

	return res;
}
Result_t I2C::Stop_Receive_DMA(void){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	if(_i2c -> CR2 & I2C_CR2_DMAEN){
		_RxDma -> Stop();
		_i2c -> CR2 &=~ I2C_CR2_DMAEN;
	}
	else{
		res.Status = ERR;
		res.CodeLine = __LINE__;
	}

	return res;
}

Result_t I2C::Memory_Transmit(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	res = SendStart();
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = SendSlaveAddr(Slave_Address, I2C_WRITE);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	if(MemAddrSize == 1U) {
		res = Transmit((uint8_t)(MemAddr & 0x00FF)); // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	else{
		res = Transmit((uint8_t)((MemAddr & 0x00FF) >> 8U)); // MEMORY ADDRESS MSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
		res = Transmit((uint8_t)(MemAddr & 0x00FF));         // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	res = Transmit(Data, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	SendStop();

	return res;
}

Result_t I2C::Memory_Receive(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};

	res = SendStart();
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = SendSlaveAddr(Slave_Address, I2C_WRITE);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	if(MemAddrSize == 1U) {
		res = Transmit((uint8_t)(MemAddr & 0x00FF)); // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	else{
		res = Transmit((uint8_t)((MemAddr & 0x00FF) >> 8U)); // MEMORY ADDRESS MSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
		res = Transmit((uint8_t)(MemAddr & 0x00FF));         // MEMORY ADDRESS LSB.
		if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	}
	res = SendRepeatStart();
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = SendSlaveAddr(Slave_Address, I2C_READ);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	res = Receive(Data, Size);
	if(res.Status != OKE){ res.CodeLine = __LINE__; return res; }
	SendStop();

	return res;
}



/* Result_t I2C::Master_Transmit(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t TxCount = Size;

	// WAIT I2C NOT BUSY.
	res = WaitBusy();
	if(res.Status != READY){
		res.CodeLine = __LINE__;
		return res;
	}
	// DISABLE POS.
	_i2c -> CR1 &=~ I2C_CR1_POS;
	// GENERATE START.
	_i2c -> CR1 |= I2C_CR1_START;
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}

	// SEND SLAVE ADDRESS.
	if(_addrmode == I2C_ADDRESS_7BIT) // MODE 7BIT ADDRESS.
		(_i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) & ~I2C_OAR1_ADD0)); // SEND 7BIT DATA WITH LSB BIT IS RESET(TRANSMIT MODE).
	// WAIT ADDR FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_ADDR, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
		}
		res.CodeLine = __LINE__;
		return res;
	}
	// CLEAR ADDR FLAG.
	__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2; //6475
	(void)tmp;

	// WAIT TXE FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
			_i2c -> CR1 |= I2C_CR1_STOP;
		}
		res.CodeLine = __LINE__;
		return res;
	}
	// SEND MEMORY ADDR.
	if(MemAddrSize == 1U) _i2c -> DR = (uint8_t)(MemAddr & 0x00FF); // MEMORY ADDRESS LSB.
	else{
		_i2c -> DR = (uint8_t)((MemAddr & 0x00FF) >> 8U); // MEMORY ADDRESS MSB.
		// WAIT TXE FLAG IS SET.
		res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
		if(res.Status != OKE){
			if(res.Status == ERR){
				_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
				_i2c -> CR1 |= I2C_CR1_STOP;
			}
			res.CodeLine = __LINE__;
			return res;
		}
		_i2c -> DR = (uint8_t)(MemAddr & 0x00FF); // MEMORY ADDRESS LSB.
	}

	// TRANSMIT DATA.
	while(TxCount > 0U){
		// WAIT TXE FLAG IS SET.
		res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
		if(res.Status != OKE){
			if(res.Status == ERR){
				_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
				_i2c -> CR1 |= I2C_CR1_STOP;
			}
			res.CodeLine = __LINE__;
			return res;
		}
		// WRITE DATA TO DR.
		_i2c -> DR = *Data++;
		TxCount--;
		if((_i2c -> SR1 & I2C_SR1_BTF) && TxCount != 0U){
			// WRITE DATA TO DR.
			_i2c -> DR = *Data++;
			TxCount--;
		}
	}// 2624.
	// WAIT BTF FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
		}
		res.CodeLine = __LINE__;
		return res;
	}

	// GENERATE STOP.
	_i2c -> CR1 |= I2C_CR1_STOP;

	return res;
}
*/

/* Result_t I2C::Master_Receive(uint16_t Slave_Address, uint16_t MemAddr, uint8_t MemAddrSize, uint8_t *Data, uint16_t Size){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	uint32_t RxCount = Size;

	// WAIT I2C NOT BUSY.
	res = WaitBusy();
	if(res.Status != READY){
		res.CodeLine = __LINE__;
		return res;
	}
	// DISABLE POS.
	_i2c -> CR1 &=~ I2C_CR1_POS;
	// ENABLE ACK.
	_i2c -> CR1 |= I2C_CR1_ACK;

	// GENERATE START.
	_i2c -> CR1 |= I2C_CR1_START;
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}

	// SEND SLAVE ADDRESS.
	if(_addrmode == I2C_ADDRESS_7BIT) // MODE 7BIT ADDRESS.
		(_i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) & ~I2C_OAR1_ADD0)); // SEND 7BIT DATA WITH LSB BIT IS RESET(TRANSMIT MODE).
	// WAIT ADDR FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_ADDR, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
		}
		res.CodeLine = __LINE__;
		return res;
	}
	// CLEAR ADDR FLAG.
	__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2; //6475
	(void)tmp;

	// WAIT TXE FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
			_i2c -> CR1 |= I2C_CR1_STOP;
		}
		res.CodeLine = __LINE__;
		return res;
	}
	// SEND MEMORY ADDR.
	if(MemAddrSize == 1U) _i2c -> DR = (uint8_t)(MemAddr & 0x00FF); // MEMORY ADDRESS LSB.
	else{
		_i2c -> DR = (uint8_t)((MemAddr & 0x00FF) >> 8U); // MEMORY ADDRESS MSB.
		// WAIT TXE FLAG IS SET.
		res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
		if(res.Status != OKE){
			if(res.Status == ERR){
				_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
				_i2c -> CR1 |= I2C_CR1_STOP;
			}
			res.CodeLine = __LINE__;
			return res;
		}
		_i2c -> DR = (uint8_t)(MemAddr & 0x00FF); // MEMORY ADDRESS LSB.
	}
	// WAIT TXE FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
			_i2c -> CR1 |= I2C_CR1_STOP;
		}
		res.CodeLine = __LINE__;
		return res;
	}

	// GENERATE RESTART.
	_i2c -> CR1 |= I2C_CR1_START;
	res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}

	// SEND SLAVE ADDRESS READ.
	if(_addrmode == I2C_ADDRESS_7BIT) // MODE 7BIT ADDRESS.
		(_i2c -> DR = (uint8_t)((Slave_Address & 0x00FF) | I2C_OAR1_ADD0)); // SEND 7BIT DATA WITH LSB BIT IS SET(RECEIVE MODE).
	// WAIT ADDR FLAG IS SET.
	res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_ADDR, FLAG_SET, I2C_DEFAULT_TIMEOUT);
	if(res.Status != OKE){
		if(res.Status == ERR){
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
		}
		res.CodeLine = __LINE__;
		return res;
	}

	// SETUP BEFORE READ DATA.
	if(RxCount == 0U){
		// CLEAR ADDR FLAG.
		__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2; //6475
		(void)tmp;
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 1U){
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
//		__disable_irq();
		// CLEAR ADDR FLAG.
		__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2; //6475
		(void)tmp;
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
//		__enable_irq();
	}
	else if(RxCount == 2U){
		// ENABLE POS.
		_i2c -> CR1 |= I2C_CR1_POS;
		__disable_irq();
		// CLEAR ADDR FLAG.
		__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2; //6475
		(void)tmp;
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
		__enable_irq();
	}
	else{
		// ENABLE ACK.
		_i2c -> CR1 |= I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2; //6475
		(void)tmp;
	}
	// START READ DATA.
	while(RxCount > 0U){
		if(RxCount <= 3U){
			if(RxCount == 1U){
				// WAIT RXNE IS SET.
				res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE){
					if(res.Status == ERR) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
					res.CodeLine = __LINE__;
					return res;
				}
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
			else if(RxCount == 2U){
				// WAIT BTF FLAG IS SET.
				res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE){
					res.CodeLine = __LINE__;
					return res;
				}
				__disable_irq();
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				__enable_irq();
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;

			}
			else{
				// WAIT BTF FLAG IS SET.
				res = WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE){
					res.CodeLine = __LINE__;
					return res;
				}
				// DISABLE ACK.
				_i2c -> CR1 &=~ I2C_CR1_ACK;
				__disable_irq();
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// WAIT BTF FLAG IS SET.
				res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_DEFAULT_TIMEOUT);
				if(res.Status != OKE){
					__enable_irq();
					res.CodeLine = __LINE__;
					return res;
				}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				__enable_irq();
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
		else{
			// WAIT RXNE IS SET.
			res = CheckFlag_In_WaitFlagTimeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_DEFAULT_TIMEOUT);
			if(res.Status != OKE){
				if(res.Status == ERR){
					_i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
				}
				res.CodeLine = __LINE__;
				return res;
			}
			// READ FORM DR.
			*Data++ = (uint8_t)_i2c -> DR;
			RxCount--;
			if(_i2c -> SR1 & I2C_SR1_BTF){
				// READ FORM DR.
				*Data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
	}

	return res;
}
*/







