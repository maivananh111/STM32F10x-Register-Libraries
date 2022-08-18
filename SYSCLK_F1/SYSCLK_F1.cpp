/*
 * SYSCLK_F1.c
 *
 *  Created on: May 30, 2022
 *      Author: A315-56
 */

#include "SYSCLK_F1.h"
#include "math.h"
#include "STM_LOG.h"
#include "stm32f103xb.h"

volatile uint32_t Tick;
static SysClockMux Mux = HSI;
static SysClockSource CLKSource = HSI_CRYSTAL;
static uint32_t SYSClock = 0U;
static uint32_t PLL_mul  = 0U;
static uint32_t AHB_psc  = 0U;
static uint32_t APB1_psc = 0U;
static uint32_t APB2_psc = 0U;

void SystemClockInit(SysClockSource Source, RCC_Config conf){
	CLKSource = Source;
	SYSClock = conf.SYSCLK;
	Mux      = conf.CLKMUX;
	AHB_psc  = conf.AHBPSC;
	APB1_psc = conf.APB1PSC;
	APB2_psc = conf.APB2PSC;

	RCC -> APB2ENR |= RCC_APB2ENR_IOPDEN;
	if(CLKSource == HSI_CRYSTAL){
		PLL_mul = (uint32_t)(SYSClock / (HSI_VALUE/2)) - 2;
		if(PLL_mul > 16UL) PLL_mul = 16UL;
		/* ENABLE HSI OCSILLATOR */
		RCC -> CR |= RCC_CR_HSION;
		while(!(RCC -> CR & RCC_CR_HSIRDY));
		/* HSI DEFAULT CALIBRATION TRIMMING VALUE */
		RCC -> CR |= (HSI_TRIM_VALUE << RCC_CR_HSITRIM_Pos);
	}
	else if(CLKSource == HSE_CRYSTAL){
		PLL_mul = (uint32_t)(SYSClock / HSE_VALUE) - 2;
		if(PLL_mul > 16UL) PLL_mul = 16UL;
		/* ENABLE HSE OCSILLATOR */
		RCC -> CR |= RCC_CR_HSEON;
		while(!(RCC -> CR & RCC_CR_HSERDY));
	}

	/* ENABLE POWER INTERFACE CLOCK */
	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;

	/* FLASH LATENCY 2WS, PREFETCH BUFER ENABLE, DATA CACHE ENABLE */
	FLASH -> ACR |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE;
	while(!(FLASH -> ACR & FLASH_ACR_PRFTBS));

	/* SET PLLMUL x16, AHB, APB1, APB2 CLOCK DIV */
	RCC -> CFGR |= AHB_psc | APB1_psc | APB2_psc;
	if(Mux == PLLCLK) RCC -> CFGR |= (PLL_mul << RCC_CFGR_PLLMULL_Pos);

	/* SET PLLCLK SOURCE */
	if(CLKSource == HSI_CRYSTAL && Mux == PLLCLK)      RCC -> CFGR &=~ RCC_CFGR_PLLSRC; /* PLL CLOCK SOURCE HSI DIV2 */
	else if(CLKSource == HSE_CRYSTAL && Mux == PLLCLK) RCC -> CFGR |= RCC_CFGR_PLLSRC;  /* PLL CLOCK SOURCE HSE DIV1 */

	/* SET SYSCLK FORM SYSTEM CLOCK MUX */
	if(Mux == PLLCLK){
		/* ENABLE PLL CLOCK */
		RCC -> CR |= RCC_CR_PLLON;
		while(!(RCC -> CR & RCC_CR_PLLRDY));
		/* SELECT SW SYSTEM CLOCK IS PLL CLOCK */
		RCC -> CFGR |= RCC_CFGR_SW_PLL;
		while(!(RCC -> CFGR & RCC_CFGR_SWS_PLL));
	}
	else if(Mux == HSE){
		/* SELECT SW SYSTEM CLOCK IS HSE */
		RCC -> CFGR |= RCC_CFGR_SW_HSE;
		while(!(RCC -> CFGR & RCC_CFGR_SWS_HSE));
	}
	else{
		/* SELECT SW SYSTEM CLOCK IS HSI */
		RCC -> CFGR &=~ RCC_CFGR_SW;
		while((RCC -> CFGR & RCC_CFGR_SW));
	}

	SystemInit();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000U);

	/* ENABLE AFIO CLOCK */
	RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;
#ifdef DEBUG_DISABLE_JTAG
	/* DISABLE JTAG */
	AFIO -> MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
#endif

}

uint32_t GetBusFreq(PCLKBus Bus){
	switch(Bus){
		case SYSCLK:
			if(CLKSource == HSE_CRYSTAL){ // HSE.
				if(Mux == HSE) return (uint32_t)HSE_VALUE;
				else if(Mux == PLLCLK) return (uint32_t)(HSE_VALUE * (PLL_mul + 2U));
			}
			else{ // HSI.
				if(Mux == HSI) return (uint32_t)HSI_VALUE;
				else if(Mux == PLLCLK) return (uint32_t)((HSI_VALUE / 2U) * (PLL_mul + 2U));
			}
		break;

		case HCLK:
			return (uint32_t)SystemCoreClock;
		break;

		case PCLK1:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
		break;

		case PCLK2:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
		break;
	}
	return 0;
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

void STM_Restart(uint16_t Time){
	for(uint16_t i=0; i<Time; i++){
		STM_LOG(BOLD_RED, "SYSTEM", "Chip will restart after %d seconds...", i+1);
		Tick_Delay_ms(1000);
	}
	__NVIC_SystemReset();
}


