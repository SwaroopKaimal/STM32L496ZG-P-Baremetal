/*
 * stm32l496xx_gpio.c
 *
 *  Created on: Jun 11, 2025
 *      Author: Swaroop Kaimal
 */

#include "stm32l496xx_gpio.h"
#include "stm32l496xx.h"

/***************************Documentation**************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @brieF 		- GPIO initialization function
 *
 * @param[in] 	- Base address of GPIO peripheral
 * @param[in] 	- Function Enable or Disable (0 or 1)
 * @param[in] 	- .
 *
 * return 		- void
 *
 * @Note:		-
 *****************************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}

	}
}
/*Initializing GPIO with user's required configurations*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t  temp=0; //Temporary register

	//1. Configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//Non-interrupt modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //2 bits for each pin number so multiply by 2
		//Clear required bit fields
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//Now store into actual register
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else{
		//Code later - Lecture 108

		/*IMPORTANT Change - To make sure that pin is configured as input, in L496 default is  Analog (0xFFFFFFFF)
		 * for other micro-controllers default might already be in input mode (0x00000000) ************************/

		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		/**********************************************************************************************************/

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			/*1. Configure FTSR1*/
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR1 bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			/*1. Configure RTSR1*/
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR1 bit
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT)
		{
			/*1. Configure both FTSR and RTSR, set both*/
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/*2. Configure the GPIO port selection in SYSCGF_EXTICR*/
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		/*3. Enable EXTI interrupt delivery mode using IMR */
		EXTI->IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//Reset value of temp
	temp=0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;

	//3. Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;

	//4. Configure the OPType
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;

	//5. Configure AF
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1, temp2; //Check Theory
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4* temp2)); //Clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2)); //Setting
	}


}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	//Right shifting the required pin state bit to LSB and then masking the remaining bits
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	//NVIC Registers data available in Cortex M4 generic user guide
	//Interrupt Set-Enable Registers (ISER) , Interrupt Clear-Enable Registers(ICER) and Interrupt Priority Registers (IPR) are to be used.

	if(EnorDi == ENABLE){

		if(IRQNumber <= 31){
			/*Program ISER0 Register*/
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 64){
			/*Program ISER1 Register*/
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber > 64 && IRQNumber <= 96){
			/*Program ISER2 Register - L496ZG-P has only 90 IRQs implemented*/
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}

	}else{
		if(IRQNumber <= 31){
			/*Program ICER0 Register*/
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 64){
			/*Program ICER1 Register*/
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber > 64 && IRQNumber <= 96){
			/*Program ICER2 Register*/
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_bits = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //NO_PR_BITS_IMPLEMENTED is usually upper 4 bits of each section in STM
	*(NVIC_PR_BASEADDR + (4 * iprx)) |=  (IRQPriority << shift_bits);

	/*Interrupt Priority when more IRQs have the same Priority
	 * Refer the below the web source for information:
	 * https://community.arm.com/arm-community-blogs/b/embedded-and-microcontrollers-blog/posts/cutting-through-the-confusion-with-arm-cortex-m-interrupt-priorities
	 * */
}

void GPIO_IRQHandling(uint8_t PinNumber){

	 /*Clear the EXTI PR register for the corresponding GPIO pin
	  * Actual ISR code has to be executed in the main user application code,
	  * over riding the .weak ISR implementations is the device startup code.*/

	if(EXTI->PR1 & (1 << PinNumber))
	{
		// This bit is cleared by writing a ‘1’ to the bit. As per RM
		EXTI->PR1 |= (1 << PinNumber);
	}

}

