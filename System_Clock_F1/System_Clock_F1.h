/*
 * System_Clock_F1.h
 *
 *  Created on: May 30, 2022
 *      Author: A315-56
 */

#ifndef SYSTEM_CLOCK_F1_H_
#define SYSTEM_CLOCK_F1_H_


#include "stdio.h"


#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TICK 0xFFFFFFFFU


void SystemClock_HSE_Init(void);
void SysTick_Handler(void);
uint32_t GetTick(void);
void Tick_Delay_ms(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_CLOCK_F1_H_ */
