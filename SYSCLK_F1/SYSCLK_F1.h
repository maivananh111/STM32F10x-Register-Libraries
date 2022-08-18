/*
 * SYSCLK_F1.h
 *
 *  Created on: May 30, 2022
 *      Author: A315-56
 */

#ifndef SYSCLK_F1_H_
#define SYSCLK_F1_H_


#include "stdio.h"

#include "stm32f1xx.h"

#ifdef __cplusplus
extern "C" {
#endif


#if !defined  (HSE_VALUE)
  #define HSE_VALUE               8000000U
#endif

#if !defined  (HSI_VALUE)
  #define HSI_VALUE               8000000U
#endif

#define MAX_TICK 0xFFFFFFFFU
#define HSI_TRIM_VALUE 16UL

#define DEBUG_DISABLE_JTAG

typedef enum{
	HSI_CRYSTAL,
	HSE_CRYSTAL,
} SysClockSource;

typedef enum{
	HSI = RCC_CFGR_SW_HSI,
	HSE = RCC_CFGR_SW_HSE,
	PLLCLK = RCC_CFGR_SW_PLL,
} SysClockMux;

typedef enum{
	SYSCLK,
	HCLK,
	PCLK1,
	PCLK2,
} PCLKBus;


typedef struct{
	SysClockMux CLKMUX;
	uint32_t SYSCLK;
	uint32_t AHBPSC;
	uint32_t APB1PSC;
	uint32_t APB2PSC;
} RCC_Config;

void SystemClockInit(SysClockSource Source, RCC_Config conf);
uint32_t GetBusFreq(PCLKBus Bus);
void SysTick_Handler(void);
uint32_t GetTick(void);
void Tick_Delay_ms(uint32_t delay_ms);

void STM_Restart(uint16_t Time);

#ifdef __cplusplus
}
#endif

#endif /* SYSCLK_F1_H_ */
