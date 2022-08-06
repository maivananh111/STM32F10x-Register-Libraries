/*
 * ADC_F1.cpp
 *
 *  Created on: Jun 30, 2022
 *      Author: A315-56
 */

#include "stm32f1xx.h"
#include "ADC_F1.h"
#include "GPIO_F1.h"



void ADC_Init(ADC_TypeDef *ADC, uint8_t channel[], GPIO_TypeDef *port_channel[], uint8_t num_cvt){
	uint32_t tmp_ADC_SMPR1 = 0;
	uint32_t tmp_ADC_SMPR2 = 0;
	/* ADC CLOCK */
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;
//	RCC -> APB2ENR |= RCC_APB2ENR_ADC2EN;
	/* GPIO SETUP AT ANALOG MODE */
	for(uint8_t i=0; i<num_cvt; i++) GPIO_Analog(port_channel[i], channel[i]);
	/* ADC PRESCALER DIV 6 */
	RCC -> CFGR |= RCC_CFGR_ADCPRE_DIV6;
	/* ADC CONFIGURATION */
	ADC -> CR1 |= ADC_CR1_SCAN;   // ADC SCAN MODE
	ADC -> CR2 |= ADC_CR2_CONT;   // ADC CONTINUOUS MODE
	ADC -> CR2 |= ADC_CR2_EXTSEL; // ADC START BY SWSTART
	ADC -> CR2 &=~ ADC_CR2_ALIGN; // ADC DATA ALIGN RIGHT

	for(uint8_t i=0; i<num_cvt; i++){
		if(channel[i] <= 9) tmp_ADC_SMPR2 |= (ADC_SMP << (3*channel[i])); // SMPR1 REGISTER
		else 				tmp_ADC_SMPR1 |= (ADC_SMP << (3*(channel[i] - 10))); // SMPR1 REGISTER
	}
	ADC -> SMPR1 = tmp_ADC_SMPR1; // ADC CHANNEL 10-17 SAMPLING TIME
	ADC -> SMPR2 = tmp_ADC_SMPR2; // ADC CHANNEL 0 - 9 SAMPLING TIME


	uint8_t num_reg = num_cvt/6;
	uint32_t ADC_SQR1 = 0, ADC_SQR2 = 0, ADC_SQR3 = 0;
	if(num_reg == 0) num_reg++;
	if(num_reg == 1 && num_cvt <= 6){
		for(uint8_t i=0; i<num_cvt; i++) ADC_SQR3 |= (channel[i] << i*5);
	}
	else if(num_reg == 2 && num_cvt <=12){
		for(uint8_t i=0; i<6; i++)       ADC_SQR3 |= (channel[i] << i*5); //0 -> 5
		for(uint8_t i=6; i<num_cvt; i++) ADC_SQR2 |= (channel[i] << (i-6)*5); // 6 -> numcvt
	}
	else{
		for(uint8_t i=0; i<6; i++)       ADC_SQR3 |= (channel[i] << i*5); //0 -> 5
		for(uint8_t i=6; i<12; i++)      ADC_SQR2 |= (channel[i] << (i-6)*5); // 6 -> 11
		for(uint8_t i=12; i<num_cvt; i++)ADC_SQR1 |= (channel[i] << (i-12)*5); // 12 -> numcvt
	}
	ADC -> SQR1 = ADC_SQR1; // ADC SEQUENCE NUMBER
	ADC -> SQR2 = ADC_SQR2; // ADC SEQUENCE NUMBER
	ADC -> SQR3 = ADC_SQR3; // ADC SEQUENCE NUMBER

	ADC -> SQR1 |= ((num_cvt-1) << ADC_SQR1_L_Pos); // ADC NUMBER OF CONVERSION SEQUENCE

	ADC -> CR2 |= ADC_CR2_ADON; // TURN ON ADC AND TO START CONVERSION
}

void ADC_Start(ADC_TypeDef *ADC){
	ADC -> CR2 |= ADC_CR2_ADON; // ENABLE ADC
	ADC -> SR = 0; // CLEAR ADC STATUS REGISTER
	ADC -> CR2 |= ADC_CR2_EXTTRIG; // ENABLE ADC START BY EXTERNAL TRIGER
	ADC -> CR2 |= ADC_CR2_SWSTART; // START ADC
}

void ADC_Stop(ADC_TypeDef *ADC){
	ADC -> CR2 &=~ ADC_CR2_ADON; // DISABLE ADC
	ADC -> CR2 &=~ ADC_CR2_EXTTRIG; // DISABLE ADC START BY EXTERNAL TRIGER
	ADC -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC
}

uint16_t ADC_Read(ADC_TypeDef *ADC){
	return ADC -> DR;
}

void ADC_Start_DMA(ADC_TypeDef *ADC, DMA dma, uint16_t *adc_data, uint16_t num_channel){
	ADC -> CR2 |= ADC_CR2_DMA; // ENABLE ADC DMA
	dma.Start((uint32_t)&ADC -> DR, (uint32_t)adc_data,  num_channel); // SETUP DMA
	ADC -> SR = 0; // CLEAR ADC STATUS REGISTER
	ADC -> CR2 |= ADC_CR2_EXTTRIG; // ENABLE ADC START BY EXTERNAL TRIGER
	ADC -> CR2 |= ADC_CR2_SWSTART; // START ADC
}

void ADC_Stop_DMA(ADC_TypeDef *ADC, DMA dma){
	ADC -> CR2 &=~ ADC_CR2_DMA; // DISABLE ADC DMA
	dma.Stop(); // STOP DMA
	ADC -> CR2 &=~ ADC_CR2_EXTTRIG;
	ADC -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC
}





