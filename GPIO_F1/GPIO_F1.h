/*
 * GPIO.h
 *
 *  Created on: Jun 1, 2022
 *      Author: A315-56
 */

#ifndef GPIO_F1_H_
#define GPIO_F1_H_

#include "stdio.h"
#include "stm32f1xx.h"
#include "stm32f103xb.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	GPIO_Input_Floating = 0,
	GPIO_Input_PullUp,
	GPIO_Input_PullDown,

	GPIO_Output_OpenDrain,
	GPIO_Output_PushPull,
} GPIO_NORMAL_MODE;

typedef enum{
	GPIO_AF_PushPull = 0,
	GPIO_AF_OpenDrain
} GPIO_AF_MODE;

typedef enum{
	SPI1_Remap 			= AFIO_MAPR_SPI1_REMAP,
	I2C1_Remap 			= AFIO_MAPR_I2C1_REMAP,
	USART1_Remap 		= AFIO_MAPR_USART1_REMAP,
	USART2_Remap 		= AFIO_MAPR_USART2_REMAP,
	USART3_Remap_PC 	= AFIO_MAPR_USART3_REMAP_PARTIALREMAP,
	USART3_Remap_PD 	= AFIO_MAPR_USART3_REMAP_FULLREMAP,
	TIM1_Partial_Remap  = AFIO_MAPR_TIM1_REMAP_PARTIALREMAP,
	TIM1_Full_Remap 	= AFIO_MAPR_TIM1_REMAP_FULLREMAP,
	TIM2_Partial_Remap1 = AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1,
	TIM2_Partial_Remap2 = AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2,
	TIM2_Full_Remap     = AFIO_MAPR_TIM2_REMAP_FULLREMAP,
	TIM3_Partial_Remap  = AFIO_MAPR_TIM3_REMAP_PARTIALREMAP,
	TIM3_Full_Remap     = AFIO_MAPR_TIM3_REMAP_FULLREMAP,
	TIM4_Remap     		= AFIO_MAPR_TIM4_REMAP,
	CAN_Remap_PB 		= AFIO_MAPR_CAN_REMAP_REMAP2,
	CAN_Remap_PD 		= AFIO_MAPR_CAN_REMAP_REMAP3,
} GPIO_Periph_Remap;

void GPIO_Init(void);
void GPIO_Mode(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_NORMAL_MODE mode);
void GPIO_AFOutput(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_AF_MODE mode);
void GPIO_Analog(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

void GPIO_Pullup(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);
void GPIO_Pulldown(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

void GPIO_Set(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);
void GPIO_Reset(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

int GPIO_Read(GPIO_TypeDef *gpio_port, uint16_t gpio_pin);

void GPIO_Remap(GPIO_Periph_Remap remap);

#ifdef __cplusplus
}
#endif


#endif /* GPIO_F1_H_ */
