/*
 * MVA_DEFINE.c
 *
 *  Created on: 8 thg 8, 2022
 *      Author: A315-56
 */

#include "MVA_DEF.h"
#include "SYSCLK_F1.h"

void WaitFlag(__IO uint32_t Register, uint32_t Flag, uint8_t Level){
	while((Level)? (!(Register & Flag)) : (Register & Flag));
}

Result_t WaitFlagTimeout(__IO uint32_t *Register, uint32_t Flag, Level_t Level, uint16_t Timeout){
	Result_t res = {
		.Status = OKE,
		.Time = 0UL,
		.CodeLine = 0,
	};
	__IO uint32_t tick = GetTick();
	while((Level == FLAG_RESET)?((*Register & Flag)) : (!(*Register & Flag))){
		res.Time   = GetTick() - tick;
		if(Timeout != NO_TIMEOUT){
			if(res.Time > Timeout) {
				res.Status = TIMEOUT;
				res.Time   = Timeout;
				return res;
			}
		}
	}
	return res;
}

Result_t CheckFlag_In_WaitFlagTimeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, Level_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, Level_t LevelWait,
										uint16_t Timeout){
	Result_t res = {
		.Status = OKE,
		.Time = 0UL,
		.CodeLine = 0,
	};
	__IO uint32_t tick = GetTick();
	while((LevelWait == FLAG_RESET)? (*RegisterWait & FlagWait) : (!(*RegisterWait & FlagWait))){
		res.Time   = GetTick() - tick;
		if((LevelCheck == FLAG_RESET)? (!(*RegisterCheck & FlagCheck)) : (*RegisterCheck & FlagCheck)) {
			res.Status = ERR;
			return res;
		}
		if(Timeout != NO_TIMEOUT){
			if(res.Time > Timeout) {
				res.Status = TIMEOUT;
				res.Time   = Timeout;
				return res;
			}
		}
	}

	return res;
}


