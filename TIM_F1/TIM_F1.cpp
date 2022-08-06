/*
 * TIM_F1.cpp
 *
 *  Created on: 9 thg 6, 2022
 *      Author: A315-56
 */

#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include "GPIO_F1.h"
#include "TIM_F1.h"


TIM::TIM(TIM_TypeDef *Timer, TIM_Direction Direction, TIM_Auto_RePreLoad ARPE){
	_tim = Timer;
	_dir = Direction;
	_arpe = ARPE;
	if(_tim == TIM1){
		_tim_num = 1;
	}
	else if(_tim == TIM2){
		_tim_num = 2;
	}
	else if(_tim == TIM3){
		_tim_num = 3;
	}
	else if(_tim == TIM4){
		_tim_num = 4;
	}
}

TIM::TIM(TIM_TypeDef *Timer, TIM_Channel Channel){
	_tim = Timer;
	_channel = Channel;
	if(_tim == TIM1){
		_tim_num = 1;
	}
	else if(_tim == TIM2){
		_tim_num = 2;
	}
	else if(_tim == TIM3){
		_tim_num = 3;
	}
	else if(_tim == TIM4){
		_tim_num = 4;
	}
}

/* TIM Basic */
void TIM::Base_Init(uint16_t psc, uint16_t arr){
	if(_tim_num == 1) RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	else RCC -> APB1ENR |= (1<<(_tim_num - 2));

	_tim -> CR1 |= (_dir << TIM_CR1_DIR_Pos) | (_arpe << TIM_CR1_ARPE_Pos);

	_tim -> ARR = arr - 1;
	_tim -> PSC = psc - 1;

	_tim -> CR1 |= TIM_CR1_CEN;
	while (!(_tim -> SR & TIM_SR_UIF));
}

void TIM::Start_DMA(DMA dma, uint16_t *count_buf, uint16_t size){
	_tim -> CR1  &=~ TIM_CR1_CEN;
	_tim -> DIER &=~ TIM_DIER_CC1DE;
	dma.Start((uint32_t)count_buf, (uint32_t)&_tim -> CNT, size);
	_tim -> DIER |= TIM_DIER_CC1DE;
	_tim -> CR1  |= TIM_CR1_CEN;
}

void TIM::ResetCounter(void){
	_tim -> CNT = 0;
}

uint16_t TIM::GetCounter(void){
	return _tim -> CNT;
}

void TIM::Delay_us(uint16_t us){
	_tim -> CNT = 0;
	while(_tim -> CNT < us);
}

void TIM::Delay_ms(uint16_t ms){
	for (uint16_t i=0; i<ms; i++)
		Delay_us(1000);
}


/* TIMER Mode PWM */
void TIM::PWM_Init(TIM_Channel Channel, GPIO_TypeDef *ChannelPort, uint16_t ChannelPin, TIM_PWMMode PWMMode, uint16_t psc, uint16_t arr){
	if(_tim_num == 1) RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	else RCC -> APB1ENR |= (1<<(_tim_num - 2));

	if(ChannelPort == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	if(ChannelPort == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	if(ChannelPort == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIO_AFOutput(ChannelPort, ChannelPin, GPIO_AF_PushPull);

	_tim -> CR1 |= (_dir << TIM_CR1_DIR_Pos) | (_arpe << TIM_CR1_ARPE_Pos);

	_tim -> ARR = arr - 1;
	_tim -> PSC = psc - 1;

	if(Channel < TIM_Channel3){ // Channel 1-2
		_tim -> CCMR1 |=  (TIM_CCMR1_OC1PE << (Channel*8)); // Set PE.
		_tim -> CCMR1 &=~ (TIM_CCMR1_OC1FE << (Channel*8)); // Clear FE.
		_tim -> CCMR1 |=  ((PWMMode << TIM_CCMR1_OC1M_Pos) << (Channel*8)); // Set 6UL to OCxM.
	}
	else{ // Channel 3-4
		_tim -> CCMR2 |=  (TIM_CCMR2_OC3PE << ((Channel - 2)*8)); // Set PE.
		_tim -> CCMR2 &=~ (TIM_CCMR2_OC3FE << ((Channel - 2)*8)); // Clear FE.
		_tim -> CCMR2 |=  ((PWMMode << TIM_CCMR2_OC3M_Pos) << ((Channel - 2)*8)); // Set PWM mode1 or mode2 (mode1 notinvert: 6, mode2 invert: 7) to OCxM.
	}

	_tim -> CCER |= (TIM_CCER_CC1E << (Channel*4));

	_tim -> CR1 |= TIM_CR1_CEN;
	while (!(_tim -> SR & TIM_SR_UIF));
}

void TIM::PWM_SetDuty(TIM_Channel Channel, uint16_t pwm){
	switch(Channel){
		case TIM_Channel1:
			_tim -> CCR1 = pwm;
		break;
		case TIM_Channel2:
			_tim -> CCR2 = pwm;
		break;
		case TIM_Channel3:
			_tim -> CCR3 = pwm;
		break;
		case TIM_Channel4:
			_tim -> CCR4 = pwm;
		break;
	};
}

void TIM::PWM_Start_DMA(TIM_Channel Channel, DMA dma, uint16_t *pwm, uint16_t size){
	_tim -> CCER &=~ (TIM_CCER_CC1E << (Channel*4));
	_tim -> DIER &=~ TIM_DIER_CC1DE << Channel;
	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr;
	switch(Channel){
		case TIM_Channel1:
			CCRx_addr = (uint32_t)&_tim -> CCR1;
		break;
		case TIM_Channel2:
			CCRx_addr = (uint32_t)&_tim -> CCR2;
		break;
		case TIM_Channel3:
			CCRx_addr = (uint32_t)&_tim -> CCR3;
		break;
		case TIM_Channel4:
			CCRx_addr = (uint32_t)&_tim -> CCR4;
		break;
	};
	dma.Start((uint32_t)pwm, CCRx_addr, size);

	_tim -> DIER |= TIM_DIER_CC1DE << Channel;

	_tim -> CCER |= (TIM_CCER_CC1E << (Channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;
}

void TIM::PWM_Stop_DMA(TIM_Channel Channel, DMA dma){
	_tim -> DIER &=~ TIM_DIER_CC1DE << Channel;
	dma.Stop();
	_tim -> CCER &=~ (TIM_CCER_CC1E << (Channel*4));
	_tim -> CR1  &=~ TIM_CR1_CEN;
}


/* TIM1 Encoder interface mode T1-T2 */
void TIM::Encoder_Init(GPIO_TypeDef *Port, uint16_t T1Pin, uint16_t T2Pin){
	if(_tim_num == 1) RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	else RCC -> APB1ENR |= (1<<(_tim_num - 2));

	if(Port == GPIOA) RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	if(Port == GPIOB) RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	if(Port == GPIOC) RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIO_Mode(Port, T1Pin, GPIO_Input_PullUp);
	GPIO_Mode(Port, T2Pin, GPIO_Input_PullUp);

	_tim -> ARR = 0xFFFE;
	_tim -> PSC = 0;

	_tim -> SMCR |= (3UL << TIM_SMCR_SMS_Pos);
	_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	_tim -> CCMR1 &=~ (TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);
	_tim -> CCMR1 &=~ (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);

	_tim -> CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P;
	_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	_tim -> DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;

	IRQn_Type TIM_IRQ;
	if(_tim == TIM1) TIM_IRQ = TIM1_CC_IRQn;
	if(_tim == TIM2) TIM_IRQ = TIM2_IRQn;
	if(_tim == TIM3) TIM_IRQ = TIM3_IRQn;
	if(_tim == TIM4) TIM_IRQ = TIM4_IRQn;

	__NVIC_SetPriority(TIM_IRQ, 0);
	__NVIC_EnableIRQ(TIM_IRQ);

	_tim -> CR1 |= TIM_CR1_CEN;
}

void TIM::Encoder_ISR(void){
	if(_tim -> SR & TIM_SR_CC1IF){
		_tim -> SR =~ TIM_SR_CC1IF;
		Enc_CNT = _tim -> CNT;
	}
	if(_tim -> SR & TIM_SR_CC2IF){
		_tim -> SR =~ TIM_SR_CC2IF;
		Enc_CNT = _tim -> CNT;
	}
}

int16_t TIM::Encoder_GetValue(void){
	return (int16_t)((int16_t)Enc_CNT/4);
}

#ifdef USE_TIM1_ISR
__WEAK void TIM1_CC_IRQHandler(void){}
#endif
#ifdef USE_TIM2_ISR
__WEAK void TIM2_IRQHandler(void){}
#endif
#ifdef USE_TIM3_ISR
__WEAK void TIM3_IRQHandler(void){}
#endif
#ifdef USE_TIM4_ISR
__WEAK void TIM4_IRQHandler(void){}
#endif









