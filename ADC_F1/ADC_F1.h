/*
 * ADC_F1.h
 *
 *  Created on: Jun 30, 2022
 *      Author: A315-56
 */

#ifndef ADC_F1_H_
#define ADC_F1_H_


#include "stdio.h"
#include "stm32f1xx.h"
#include "DMA_F1.h"
#include "MVA_DEF.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	ADC_1_5_CYCLES   = 0UL,  // 1.5 cycles
	ADC_7_5_CYCLES   = 1UL,  // 7.5 cycles
	ADC_13_5_CYCLES  = 2UL,  // 13.5 cycles
	ADC_28_5_CYCLES  = 3UL,  // 28.5 cycles
	ADC_41_5_CYCLES  = 4UL,  // 41.5 cycles
	ADC_55_5_CYCLES  = 5UL,  // 55.5 cycles
	ADC_71_5_CYCLES  = 6UL,  // 71.5 cycles
	ADC_239_5_CYCLES = 7UL,  // 239.5 cycles
} ADC_SamplingTime;

typedef struct{
	GPIO_TypeDef **ADC_PORT;
	uint8_t *ADC_GPIO;
	uint8_t *ADC_CHANNELRANK;
	ADC_SamplingTime *Channel_SammplingTime;
	uint8_t NumberOf_Convert;
	bool Temp_Vref_Sensor = false;
} ADC_Config;

void ADC_Init(ADC_TypeDef *ADC, ADC_Config adc_conf);
void ADC_Start(ADC_TypeDef *ADC);
void ADC_Stop(ADC_TypeDef *ADC);
uint16_t ADC_Read(ADC_TypeDef *ADC);
Result_t ADC_Start_DMA(ADC_TypeDef *ADC, DMA dma, uint16_t *data, uint16_t num_channel);
Result_t ADC_Stop_DMA(ADC_TypeDef *ADC, DMA dma);

float Internal_TemperatureSensor(ADC_TypeDef *ADC, uint16_t ITempSS_Data);

#ifdef __cplusplus
}
#endif

#endif /* ADC_F1_H_ */
