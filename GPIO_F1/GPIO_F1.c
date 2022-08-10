/*
 * GPIO_F1.c
 *
 *  Created on: Jun 1, 2022
 *      Author: A315-56
 */

#include "../GPIO_F1/GPIO_F1.h"


void GPIO_Init(void){
	/* ENABLE GPIO CLOCK */
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
	RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;
}

void GPIO_Mode(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_NORMAL_MODE mode){
	if(gpio_pin < 8) { // GPIO0-7.
		gpio_port -> CRL &=~ (3UL << (2 + gpio_pin*4)); // CLEAR CNF.

		if(mode <= GPIO_Input_PullDown) { // INPUT.
			gpio_port -> CRL &=~ (3UL << gpio_pin*4); // SET MODE INPUT.

			switch(mode){
				case GPIO_Input_Floating:
					gpio_port -> CRL |= (1UL << (2 + gpio_pin*4));
				break;
				case GPIO_Input_PullUp:
					gpio_port -> CRL |= (2UL << (2 + gpio_pin*4));
					gpio_port -> ODR |= (1UL << gpio_pin);
				break;
				case GPIO_Input_PullDown:
					gpio_port -> CRL |= (2UL << (2 + gpio_pin*4));
					gpio_port -> ODR &=~ (1UL << gpio_pin);
				break;
				default:
				break;
			}
		}

		else { // OUTPUT.
			gpio_port -> CRL |= (3UL << gpio_pin*4);// SET MODE OUTPUT HIGH SPEED.

			if(mode <= GPIO_Output_OpenDrain) gpio_port -> CRL |= (1UL << (2 + gpio_pin*4));
			else gpio_port -> CRL &=~ (3UL << (2 + gpio_pin*4));
		}
	}

	else { // GPIO8-15.
		gpio_port -> CRH &=~ (3UL << (2 + (gpio_pin-8)*4)); // CLEAR CNF.

		if(mode <= GPIO_Input_PullDown) { // INPUT.
			gpio_port -> CRH &=~ (3UL << (gpio_pin-8)*4); // SET MODE INPUT.

			switch(mode){
				case GPIO_Input_Floating:
					gpio_port -> CRH |= (1UL << (2 + (gpio_pin-8)*4));
				break;
				case GPIO_Input_PullUp:
					gpio_port -> CRH |= (2UL << (2 + (gpio_pin-8)*4));
					gpio_port -> ODR |= (1UL << gpio_pin);
				break;
				case GPIO_Input_PullDown:
					gpio_port -> CRH |= (2UL << (2 + (gpio_pin-8)*4));
					gpio_port -> ODR &=~ (1UL << gpio_pin);
				break;
				default:
				break;
			}
		}

		else { // OUTPUT.
			gpio_port -> CRH |= (3UL << (gpio_pin-8)*4);// SET MODE OUTPUT HIGH SPEED.

			if(mode <= GPIO_Output_OpenDrain) gpio_port -> CRH |= (1UL << (2 + (gpio_pin-8)*4));
			else gpio_port -> CRH &=~ (3UL << (2 + (gpio_pin-8)*4));
		}
	}
}

void GPIO_AFOutput(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_AF_MODE mode){
	if(gpio_pin < 8) { // GPIO0-7.
		gpio_port -> CRL &=~ (3UL << (2 + gpio_pin*4)); // CLEAR CNF.
		gpio_port -> CRL |= (3UL << gpio_pin*4);// SET MODE OUTPUT HIGH SPEED.
		if(mode == GPIO_AF_OpenDrain) gpio_port -> CRL |= (3UL << (2 + gpio_pin*4));
		else gpio_port -> CRL |= (2UL << (2 + gpio_pin*4));
	}

	else { // GPIO8-15.
		gpio_port -> CRH &=~ (3UL << (2 + (gpio_pin-8)*4)); // CLEAR CNF.
		gpio_port -> CRH |= (3UL << (gpio_pin-8)*4);// SET MODE OUTPUT HIGH SPEED.
		if(mode == GPIO_AF_OpenDrain) gpio_port -> CRH |= (3UL << (2 + (gpio_pin-8)*4));
		else gpio_port -> CRH |= (2UL << (2 + (gpio_pin-8)*4));

	}
}


void GPIO_Analog(GPIO_TypeDef *gpio_port, uint16_t gpio_pin){
	if(gpio_pin < 8) gpio_port -> CRL &=~ (0x0F << (4*gpio_pin));
	else             gpio_port -> CRH &=~ (0x0F << (4*(gpio_pin-8)));
}



void GPIO_Pullup(GPIO_TypeDef *gpio_port, uint16_t gpio_pin){
	gpio_port -> ODR |= (1 << gpio_pin);
}

void GPIO_Pulldown(GPIO_TypeDef *gpio_port, uint16_t gpio_pin){
	gpio_port -> ODR &=~ (1 << gpio_pin);
}

void GPIO_Set(GPIO_TypeDef *gpio_port, uint16_t gpio_pin){
	gpio_port -> BSRR |= (1 << gpio_pin);
}

void GPIO_Reset(GPIO_TypeDef *gpio_port, uint16_t gpio_pin){
	gpio_port -> BSRR |= (1 << (gpio_pin + 16));
}

int GPIO_Read(GPIO_TypeDef *gpio_port, uint16_t gpio_pin){
	return (gpio_port -> IDR >> gpio_pin) & 1UL;
}

void GPIO_Remap(GPIO_Periph_Remap remap){
	AFIO -> MAPR |= remap;
}


