/*
 * System_Clock_F1.c
 *
 *  Created on: May 30, 2022
 *      Author: A315-56
 */

#include "System_Clock_F1.h"

#include "stm32f1xx.h"
#include "stm32f103xb.h"


volatile uint32_t Tick;

void SystemClock_HSE_Init(void){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPDEN;
    /* ENABLE HSE OCSILLATOR */
	RCC -> CR |= RCC_CR_HSEON;
	while(!(RCC -> CR & RCC_CR_HSERDY));
	/* ENABLE POWER INTERFACE CLOCK */
	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;
	/* FLASH LATENCY 2WS, PREFETCH BUFER ENABLE, DATA CACHE ENABLE */
	FLASH -> ACR |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE;
	while(!(FLASH -> ACR & FLASH_ACR_PRFTBS));
//    /* HSI DEFAULT CALIBRATION TRIMMING VALUE */
//	RCC -> CR |= (HSI_TRIM_VALUE << RCC_CR_HSITRIM_Pos);
	/* SET PLLMUL x16, AHB, APB1, APB2 CLOCK DIV */
	RCC -> CFGR |= RCC_CFGR_PLLMULL9 | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
	/* PLL CLOCK SOURCE DIV1 */
	RCC -> CFGR |= RCC_CFGR_PLLSRC;
	/* ENABLE PLL CLOCK */
	RCC -> CR |= RCC_CR_PLLON;
	while(!(RCC -> CR & RCC_CR_PLLRDY));
	/* SELECT SW SYSTEM CLOCK IS PLL CLOCK */
	RCC -> CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC -> CFGR & RCC_CFGR_SWS_PLL));

	SystemInit();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000U);

	/* ENABLE AFIO CLOCK */
	RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;
	/* DISABLE JTAG */
	AFIO -> MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
}

uint32_t GetTick(void){
	return Tick;
}

void SysTick_Handler(void){
  Tick += 1;
}

void Tick_Delay_ms(uint32_t delay_ms){
	uint32_t tickstart = GetTick();
	uint32_t wait = delay_ms;

	if (wait < MAX_TICK) wait += 1UL;
	while ((GetTick() - tickstart) < wait);
}



