/*
 * MVA_DEFINE.h
 *
 *  Created on: Aug 3, 2022
 *      Author: A315-56
 */

#ifndef MVA_DEFINE_H_
#define MVA_DEFINE_H_

#include "stdio.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"


#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	ERR = 0,
	OKE,
	TIMEOUT,
	UNAVAILABLE,
	BUSY,
	READY
} Status_t;

typedef struct{
	Status_t Status;
	uint32_t Time;
	uint16_t CodeLine;
}Result_t;

typedef enum{
	FLAG_RESET,
	FLAG_SET
} Level_t;

enum{
	NO_TIMEOUT,
	LSE_CONF_TIMEOUT = 100000UL,
	LSI_CONF_TIMEOUT = 2000UL,
	DEFAULT_CONF_TIMEOUT = 1000UL,
	RSF_FLAG_TIMEOUT = 1000UL,

};

void WaitFlag(__IO uint32_t Register, uint32_t Flag, uint8_t Level);
Result_t WaitFlagTimeout(__IO uint32_t *Register, uint32_t Flag, Level_t Level, uint16_t Timeout);
Result_t CheckFlag_In_WaitFlagTimeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, Level_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, Level_t LevelWait,
										uint16_t Timeout);



#ifdef __cplusplus
}
#endif

#endif /* MVA_DEFINE_H_ */
