/*
 * stm32l496xx_gpio.h
 *
 *  Created on: Jun 11, 2025
 *      Author: Swaroop Kaimal
 */

#ifndef INC_STM32L496XX_GPIO_H_
#define INC_STM32L496XX_GPIO_H_

#include "stm32l496xx.h"

/*This PinConfig will be configured by the user*/
typedef struct
{
	uint8_t GPIO_PinNumber; /*There are only 16 pins per port so uint8_t is sufficient*/
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*Handle structure for GPIO Pin*/
typedef struct
{
	/*Pointer to hold the base address of GPIO peripheral*/
	GPIO_RegDef_t *pGPIOx; /*Holds the base address of the port to with the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*Below macros for user configuration of GPIO_PinConfig_t*/

/*GPIO Pin Numbers*/
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/*GPIO Pin Modes - From GPIOx_MODER register*/
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3

/*Modes for edge triggers*/
#define GPIO_MODE_IT_FT		4 //Custom numbers
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_FRT	6

/*GPIO output types*/
#define GPIO_PO_TYPE_PP		0
#define GPIO_PO_TYPE_OD		1

/*GPIO Output Speed Modes*/
#define GPIO_SPEED_LOW   	0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST  	2
#define GPIO_SPEED_HIGH   	3

/*GPIO PUPD Modes*/
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



/****************************************************************************************************
 *                                   APIs Supported by this GPIO Driver Code
 ****************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi); //Why GPIO_RegDef_t *pGPIOx and not handle? Base address

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); /*Can utilize the reset register in the RCC to reset all registers*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);






#endif /* INC_STM32L496XX_GPIO_H_ */
