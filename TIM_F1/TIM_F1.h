/*
 * TIM_F1.h
 *
 *  Created on: 9 thg 6, 2022
 *      Author: A315-56
 */

#ifndef TIM_F1_H_
#define TIM_F1_H_


#include "stdio.h"
#include "DMA_F1.h"
#include "MVA_DEF.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USE_TIM1_ISR
//#define USE_TIM2_ISR
//#define USE_TIM3_ISR
//#define USE_TIM4_ISR

typedef enum{
	TIM_CHANNEL1 = 0,
	TIM_CHANNEL2,
	TIM_CHANNEL3,
	TIM_CHANNEL4
} TIM_Channel;

typedef enum{
	TIM_COUNTER_UP = 0,
	TIM_COUNTER_DOWN,
} TIM_Direction;

typedef enum{
	TIM_ARP_DISABLE = 0,
	TIM_ARP_ENABLE,
} TIM_Auto_RePreLoad;

typedef enum{
	TIM_PWM_NOINVERT = 6,
	TIM_PWM_INVERT = 7,
} TIM_PWMMode;

class TIM{
	public:
		TIM(TIM_TypeDef *Timer, TIM_Direction Direction = TIM_COUNTER_UP, TIM_Auto_RePreLoad ARPE = TIM_ARP_DISABLE); // For Timer basic.
		TIM(TIM_TypeDef *Timer, TIM_Channel Channel);

		/* Basic TIMER */
		Result_t Base_Init(uint16_t psc, uint16_t arr);
		Result_t Start_DMA(DMA dma, uint16_t *count_buf, uint16_t size);
		Result_t Stop_DMA(DMA dma);
		void ResetCounter(void);
		uint16_t GetCounter(void);
		void Delay_us(uint16_t us);
		void Delay_ms(uint16_t ms);

		/* TIMER PWM Mode */
		Result_t PWM_Init(TIM_Channel Channel, GPIO_TypeDef *ChannelPort, uint16_t ChannelPin, TIM_PWMMode PWMMode, uint16_t psc, uint16_t arr);
		void PWM_SetDuty(TIM_Channel Channel, uint16_t pwm);
		Result_t PWM_Start_DMA(TIM_Channel Channel, DMA dma, uint16_t *pwm, uint16_t size);
		Result_t PWM_Stop_DMA(TIM_Channel Channel, DMA dma);

		/* TIMER Encoder Mode */
		void Encoder_Init(GPIO_TypeDef *Port, uint16_t T1Pin, uint16_t T2Pin);
		int16_t Encoder_GetValue(void);
		void Encoder_ISR(void);

	private:
		TIM_TypeDef *_tim;
		uint8_t _tim_num = 1;
		TIM_Channel _channel = TIM_CHANNEL1;
		TIM_Direction _dir;
		TIM_Auto_RePreLoad _arpe;

		volatile uint16_t Enc_CNT  = 0;
};

#ifdef USE_TIM1_ISR
void TIM1_CC_IRQHandler(void);
#endif
#ifdef USE_TIM2_ISR
void TIM2_IRQHandler(void);
#endif
#ifdef USE_TIM3_ISR
void TIM3_IRQHandler(void);
#endif
#ifdef USE_TIM4_ISR
void TIM4_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TIM_F1_H_ */
