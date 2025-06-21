/*
 * 002led_button.c
 *
 *  Created on: Jun 16, 2025
 *      Author: Swaroop Kaimal
 */

#include <string.h>
#include "stm32l496xx.h"

void delay(){
	for(uint32_t i=0; i<250000; i++);
}

int main(void){


	GPIO_Handle_t gpio_led, gpio_btn;

	/* IMPORTANT - If one parameter is not configured (Example: GPIO_PinOPType here) , the register MAY be initialized with garbage values.
	* So it is better to initialize all registers to zero beforehand*/

	memset(&gpio_led, 0, sizeof(gpio_led)); /* #include <string.h>  */
	memset(&gpio_btn, 0, sizeof(gpio_btn));

	gpio_led.pGPIOx = GPIOB;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP; //External pull up resistor for Open Drain
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpio_btn.pGPIOx = GPIOC;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP; //Applicable only when the mode is output

	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; //(L496ZG-P has Pull Down Resistor for PC13) Usually External pull up already available in Nucleo Board (Pull down in Discovery)

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpio_led);
	GPIO_Init(&gpio_btn);

	/*IRQ Configurations*/
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 15); //Optional in this application
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}

void EXTI9_5_IRQHandler(void)
{ //Over riding function from device starup file

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_8);
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_7);

}
