/*
 * 001led_toggle.c
 *
 *  Created on: Jun 13, 2025
 *      Author: Swaroop Kaimal
 */

#include "stm32l496xx.h"

void delay(){
	for(uint32_t i=0; i<500000; i++);
}

int main(void){


	GPIO_Handle_t gpio_led;

	gpio_led.pGPIOx = GPIOB;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP; //External pull up resistor for Open Drain
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&gpio_led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);
		delay();
	}

	return 0;
}
