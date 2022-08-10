/*
 * ADC_F1.cpp
 *
 *  Created on: Jun 30, 2022
 *      Author: A315-56
 */

#include "stm32f1xx.h"
#include "ADC_F1.h"
#include "GPIO_F1.h"
#include "SYSCLK_F1.h"
#include "math.h"


void ADC_Init(ADC_TypeDef *ADC, ADC_Config adc_conf){
	uint32_t tmp_ADC_SMPR1 = 0;
	uint32_t tmp_ADC_SMPR2 = 0;

	/* ADC CLOCK ENABLE */
	if(ADC == ADC1) RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;
	if(ADC == ADC2) RCC -> APB2ENR |= RCC_APB2ENR_ADC2EN;

	/* ADC PRESCALER DIVISION */
	uint32_t adc_div = ceil((float)GetBusFreq(PCLK2)/14000000.0);
	if(adc_div > 8U) adc_div = 8U;
	if(adc_div == 0U) adc_div = 2U;
	if(adc_div % 2 != 0U && adc_div != 0U) adc_div += 1;
	adc_div = (adc_div >> 1U) - 1;
	RCC -> CFGR |= (adc_div << RCC_CFGR_ADCPRE_Pos);

	/* GPIO SETUP AT ANALOG MODE */
	for(uint8_t i=0; i<adc_conf.NumberOf_Convert; i++) {
		if(adc_conf.ADC_GPIO[i] <= 15) GPIO_Analog(adc_conf.ADC_PORT[i], adc_conf.ADC_GPIO[i]);
	}

	/* ADC CONFIGURATION */
	ADC -> CR1 |= ADC_CR1_SCAN;   // ADC SCAN MODE
	ADC -> CR2 |= ADC_CR2_CONT;   // ADC CONTINUOUS MODE
	ADC -> CR2 |= ADC_CR2_EXTSEL; // ADC START BY SWSTART
	ADC -> CR2 &=~ ADC_CR2_ALIGN; // ADC DATA ALIGN RIGHT

	for(uint8_t i=0; i<adc_conf.NumberOf_Convert; i++){
		if(adc_conf.ADC_CHANNELRANK[i] <= 9) tmp_ADC_SMPR2 |= (adc_conf.Channel_SammplingTime[i] << (3*adc_conf.ADC_CHANNELRANK[i]));        // SMPR1 REGISTER
		else 				                 tmp_ADC_SMPR1 |= (adc_conf.Channel_SammplingTime[i] << (3*(adc_conf.ADC_CHANNELRANK[i] - 10))); // SMPR1 REGISTER
	}
	ADC -> SMPR1 = tmp_ADC_SMPR1; // ADC CHANNEL 10-17 SAMPLING TIME
	ADC -> SMPR2 = tmp_ADC_SMPR2; // ADC CHANNEL 0 - 9 SAMPLING TIME

	uint8_t num_reg = ceil((float)adc_conf.NumberOf_Convert/6.0);
	uint32_t ADC_SQR1 = 0, ADC_SQR2 = 0, ADC_SQR3 = 0;
	if(num_reg == 0) num_reg++;
	if(num_reg == 1 && adc_conf.NumberOf_Convert <= 6){
		for(uint8_t i=0; i<adc_conf.NumberOf_Convert; i++)  ADC_SQR3 |= (adc_conf.ADC_CHANNELRANK[i] << i*5);
	}
	else if(num_reg == 2 && adc_conf.NumberOf_Convert <=12){
		for(uint8_t i=0; i<6; i++)                          ADC_SQR3 |= (adc_conf.ADC_CHANNELRANK[i] << i*5); //0 -> 5
		for(uint8_t i=6; i<adc_conf.NumberOf_Convert; i++)  ADC_SQR2 |= (adc_conf.ADC_CHANNELRANK[i] << (i-6)*5); // 6 -> numcvt
	}
	else{
		for(uint8_t i=0; i<6; i++)                          ADC_SQR3 |= (adc_conf.ADC_CHANNELRANK[i] << i*5); //0 -> 5
		for(uint8_t i=6; i<12; i++)                         ADC_SQR2 |= (adc_conf.ADC_CHANNELRANK[i] << (i-6)*5); // 6 -> 11
		for(uint8_t i=12; i<adc_conf.NumberOf_Convert; i++) ADC_SQR1 |= (adc_conf.ADC_CHANNELRANK[i] << (i-12)*5); // 12 -> numcvt
	}
	ADC -> SQR1 = ADC_SQR1; // ADC SEQUENCE NUMBER.
	ADC -> SQR2 = ADC_SQR2; // ADC SEQUENCE NUMBER.
	ADC -> SQR3 = ADC_SQR3; // ADC SEQUENCE NUMBER.

	ADC -> SQR1 |= ((adc_conf.NumberOf_Convert - (uint8_t)1) << ADC_SQR1_L_Pos); // ADC NUMBER OF CONVERSION SEQUENCE

	if(adc_conf.Temp_Vref_Sensor == true) ADC -> CR2 |= ADC_CR2_TSVREFE;

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

Result_t ADC_Start_DMA(ADC_TypeDef *ADC, DMA dma, uint16_t *adc_data, uint16_t num_channel){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	ADC -> CR2 |= ADC_CR2_DMA; // ENABLE ADC DMA
	res = dma.Start((uint32_t)&ADC -> DR, (uint32_t)adc_data,  num_channel); // SETUP DMA
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	ADC -> SR = 0; // CLEAR ADC STATUS REGISTER
	ADC -> CR2 |= ADC_CR2_EXTTRIG; // ENABLE ADC START BY EXTERNAL TRIGER
	ADC -> CR2 |= ADC_CR2_SWSTART; // START ADC

	return res;
}

Result_t ADC_Stop_DMA(ADC_TypeDef *ADC, DMA dma){
	Result_t res = {
		.Status = OKE,
		.CodeLine = 0,
	};
	ADC -> CR2 &=~ ADC_CR2_DMA; // DISABLE ADC DMA
	res = dma.Stop(); // STOP DMA
	if(res.Status != OKE){
		res.CodeLine = __LINE__;
		return res;
	}
	ADC -> CR2 &=~ ADC_CR2_EXTTRIG;
	ADC -> CR2 &=~ ADC_CR2_SWSTART; // STOP ADC

	return res;
}

float Internal_TemperatureSensor(ADC_TypeDef *ADC, uint16_t ITempSS_Data){
	return (float)((1.43 - ((3.3/4096.0)*(float)ITempSS_Data))/0.0043) + 25.0;
//	return 357.558 - 0.187364 * ITempSS_Data;
}



