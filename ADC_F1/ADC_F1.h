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

#ifdef __cplusplus
extern "C"{
#endif

#define ADC_SMP 1UL //7.5 cycles

void ADC_Init(ADC_TypeDef *ADC, uint8_t channel[], GPIO_TypeDef *port_channel[], uint8_t num_cvt);
void ADC_Start(ADC_TypeDef *ADC);
void ADC_Stop(ADC_TypeDef *ADC);
uint16_t ADC_Read(ADC_TypeDef *ADC);
void ADC_Start_DMA(ADC_TypeDef *ADC, DMA dma, uint16_t *data, uint16_t num_channel);
void ADC_Stop_DMA(ADC_TypeDef *ADC, DMA dma);



#ifdef __cplusplus
}
#endif

#endif /* ADC_F1_H_ */
