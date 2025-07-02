/*
 * stm32l496xx.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Swaroop Kaimal
 */

#ifndef INC_STM32L496XX_H_
#define INC_STM32L496XX_H_

#include <stdint.h>

#define __vo volatile

/*Processor Specific Details - ARM Cortex Mx */
//NVIC ISERx Register Addresses (Registers data available in Cortex M4 generic user guide)
/* In the program, some macros are type casted to a pointer type after the final base address has been calculated.
 * However, macros that are not used in the program are not type casted.*/
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104) //Each 32 bit register is 4 bytes apart in hex
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C) //0x0C = 12 (in decimal)

//NVIC ICERx Register Addresses (Registers data available in Cortex M4 generic user guide)
#define NVIC_ICER0				((__vo uint32_t*) 0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*) 0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*) 0XE000E188)
#define NVIC_ICER3				((__vo uint32_t*) 0XE000E18C)

//NVIC IPRx Register Base Address (Registers data available in Cortex M4 generic user guide)
#define NVIC_PR_BASEADDR		((__vo uint32_t*) 0xE000E400)

//Number of priority bits is usually upper 4 bits of each section in STM, 3 in some TI microcontrollers
#define NO_PR_BITS_IMPLEMENTED	4

/****************************************************************************************************************/


/*Base addresses of FLASH and SRAM memories*/
#define FLASH_BASEADDR			0x08000000UL /*Check FLASH Module Organization in RM */
#define SRAM1_BASEADDR			0x20000000UL /*Check memory map in RM, 256kB*/
#define SRAM2_BASEADDR			0x20040000UL /*Either by calculation (add 256 * 1024 as hex to base address) or from the RM*/
#define ROM_BASEADDR			0x1FFF0000UL /*ROM is System Memory*/
#define SRAM 					SRAM1_BASEADDR /*SRAM1 is the main SRAM used*/

/*AHBx and APBx Bus Peripheral Base Addresses*/
#define PERIPH_BASEADDR			0x40000000UL
#define AHB1PERIPH_BASEADDR		0x40020000UL
#define AHB2PERIPH_BASEADDR		0x48000000UL
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000UL


/*Base addresses of peripherals in AHB1 Bus*/
#define DMA1_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0000)
#define DMA2_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)

/*Base addresses of peripherals in AHB2 Bus*/
#define GPIOA_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 			(AHB2PERIPH_BASEADDR + 0x2000)
#define ADC_BASEADDR			0x50040000UL

/*Base addresses of peripherals in APB1 Bus*/
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400)
#define LCD_BASEADDR			(APB1PERIPH_BASEADDR + 0x2400)
#define RTC_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define LPUART1_BASEADDR		(APB1PERIPH_BASEADDR + 0x8000)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)


/*Base addresses of peripherals in APB2 Bus*/
#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR + 0x0000)
#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x0400)
#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3800)

/*Peripheral Register definition structures*/

typedef struct
{
	__vo uint32_t MODER; 	/*Use  uint32_t as each register is 32 bits wide* Address offset: 0x00*/
	__vo uint32_t OTYPER;	/*Address Offset: 0x04*/
	__vo uint32_t OSPEEDR; 	/*Address Offset: 0x08*/
	__vo uint32_t PUPDR;		/*Address Offset: 0x0C*/
	__vo uint32_t IDR; 		/*Address Offset: 0x10*/
	__vo uint32_t ODR; 		/*Address Offset: 0x14*/
	__vo uint32_t BSRR; 		/*Address Offset: 0x18*/
	__vo uint32_t LCKR; 		/*Address Offset: 0x1C*/
	__vo uint32_t AFR[2]; 	/*Address Offset: 0x20, AFR[0]-LOW Register, AFR[1]-HIGH Register*/
	__vo uint32_t BRR; 		/*Address Offset: 0x28*/
	__vo uint32_t ASCR; 		/*Address Offset: 0x2C*/

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t ICSCR;
	__vo uint32_t CFGR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t PLLSAI1CFGR;
	__vo uint32_t PLLSAI2CFGR;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	uint32_t reserved1;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t reserved2;
	__vo uint32_t APB1RSTR1;
	__vo uint32_t APB1RSTR2;
	__vo uint32_t APB2RSTR;
	uint32_t reserved3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t reserved4;
	__vo uint32_t APB1ENR1;
	__vo uint32_t APB1ENR2;
	__vo uint32_t APB2ENR;
	uint32_t reserved5;
	__vo uint32_t AHB1SMENR;
	__vo uint32_t AHB2SMENR;
	__vo uint32_t AHB3SMENR;
	uint32_t reserved6;
	__vo uint32_t APB1SMENR1;
	__vo uint32_t APB1SMENR2;
	__vo uint32_t APB2SMENR;
	uint32_t reserved7;
	__vo uint32_t CCIPR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t CRRCR;
	__vo uint32_t CCIPR2;

}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;

}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t CFGR1;
	__vo uint32_t EXTICR[4];
	__vo uint32_t SCSR;
	__vo uint32_t CFGR2;
	__vo uint32_t SWPR;
	__vo uint32_t SKR;
	__vo uint32_t reserved;

}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;

}SPI_RegDef_t;


/*Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t */
#define GPIOA 				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*) SPI3_BASEADDR)


/*Clock Enable and Disable Macros for GPIOx*/
#define GPIOA_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 		(RCC->AHB2ENR |= (1 << 8))

#define GPIOA_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() 		(RCC->AHB2ENR &= ~(1 << 8))


/*Clock Enable and Disable Macros for I2Cx*/
#define I2C1_PCLK_EN() 		(RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR1 |= (1 << 23))

#define I2C1_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PCLK_DI() 		(RCC->APB1ENR1 &= ~(1 << 23))


/*Clock Enable and Disable Macros for SPIx*/
#define SPI1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR1 |= (1 << 15))

#define SPI1_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR1 &= ~(1 << 15))

/*Clock Enable and Disable Macros for USARTx*/
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR1 |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR1 |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR1 |= (1 << 20))
#define LPUART1_PCLK_EN() 	(RCC->APB1ENR2 |= (1 << 0))

#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR1 &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR1 &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR1 &= ~(1 << 20))
#define LPUART1_PCLK_DI() 	(RCC->APB1ENR2 &= ~(1 << 0))

/*Clock Enable Macros and Disable for SYSCFG*/
#define SYSCFG_PCLK_EN()  	(RCC->APB2ENR |= (1 << 0))
#define SYSCFG_PCLK_DI()  	(RCC->APB2ENR &= ~(1 << 0))

/*Macros to reset GPIO Peripherals*/
/* Use do while loop to implement 2 statements in same macro one after the other,
 * no need for semicolon in the macro because we use it while calling these functions in the main code*/
#define GPIOA_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 0))	; (RCC->AHB2RSTR &= ~(1 << 0)) ; }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 1))	; (RCC->AHB2RSTR &= ~(1 << 1)) ; }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 2))	; (RCC->AHB2RSTR &= ~(1 << 2)) ; }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 3))	; (RCC->AHB2RSTR &= ~(1 << 3)) ; }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 4))	; (RCC->AHB2RSTR &= ~(1 << 4)) ; }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 5))	; (RCC->AHB2RSTR &= ~(1 << 5)) ; }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 6))	; (RCC->AHB2RSTR &= ~(1 << 6)) ; }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 7))	; (RCC->AHB2RSTR &= ~(1 << 7)) ; }while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB2RSTR |= (1 << 8))	; (RCC->AHB2RSTR &= ~(1 << 8)) ; }while(0)

/*Macros to reset GPIO Peripherals*/
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)) ; (RCC->APB2RSTR &= ~(1 << 12)) ;}while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR1 |= (1 << 14)) ; (RCC->APB1RSTR1 &= ~(1 << 14)) ;}while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR1 |= (1 << 15)) ; (RCC->APB1RSTR1 &= ~(1 << 15)) ;}while(0)


/*Some extra macros*/
#define ENABLE 			1
#define SET				1
#define DISABLE 		0
#define RESET			0
#define GPIO_PIN_SET 	1
#define GPIO_PIN_RESET 	0
#define FLAG_RESET      0
#define FLAG_SET		1
#define HIGH			1
#define LOW				0

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 : 0)

/*IRQ Numbers of EXTI lines*/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/* Bit Position definitions of SPI Peripheral*/

//SPI CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSRT			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_CRCL			11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

//SPI CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_NSSP			3
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7
#define SPI_CR2_DS				8
#define SPI_CR2_FRXTH			12
#define SPI_CR2_LDMA_RX			13
#define SPI_CR2_LDMA_TX			14

//SPI Status Register
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8
#define SPI_SR_FRLVL			9
#define SPI_SR_FTLVL			11


#include "stm32l496xx_gpio.h"
#include "stm32l496xx_spi.h"

#endif /* INC_STM32L496XX_H_ */
