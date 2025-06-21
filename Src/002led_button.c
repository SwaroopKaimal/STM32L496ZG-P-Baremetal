/*
 * 002led_button.c
 *
 *  Created on: Jun 16, 2025
 *      Author: Swaroop Kaimal
 */

#include "stm32l496xx.h"

#define BTN_PRESSED 1 //Button to VCC - L496ZG-P has Pull Down Resistor for PC13
#define BTN_RELEASED 0

void delay(){
	for(uint32_t i=0; i<250000; i++);
}

int main(void){


	GPIO_Handle_t gpio_led, gpio_btn;

	gpio_led.pGPIOx = GPIOB;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP; //External pull up resistor for Open Drain
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpio_btn.pGPIOx = GPIOC;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP; //Applicable only when the mode is output
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //(Check for L496ZG - Pulled Down for PC13) Usuallu External pull up already available in Nucleo Board (Pull down in Discovery)

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpio_led);
	GPIO_Init(&gpio_btn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED)
		{
			delay(); //Delay for button de-bouncing
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
		}

	}

	return 0;
}

