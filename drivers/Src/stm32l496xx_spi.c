/*
 * stm32l496xx_spi.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Swaroop S Kaimal
 */

#include "stm32l496xx_spi.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}

	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}

	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/*Enable the peripheral clock so that its easy for the user*/
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	/* Configure the SPI_CR1 register */

	uint32_t tempreg1 = 0;

	/* 1. Configure the device mode */
	tempreg1 |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSRT);

	/* 2. Configure the bus configuration */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* Clear the BIDI mode*/
		tempreg1 &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* Enable the BIDI mode*/
		tempreg1 |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		/* CLear the BIDI mode*/
		tempreg1 &= ~(1 << SPI_CR1_BIDIMODE);

		/*RXONLY bit must be set*/
		tempreg1 |= (1 << SPI_CR1_RXONLY);
	}

	/*3. Configure the SPI serial clock speed*/
	tempreg1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	/*4. Configure the DFF*/
	uint32_t tempreg2=0;
	tempreg2 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR2_DS);

	/*5. Configure the CPOL*/
	tempreg1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	/*6. Configure the CPHA*/
	tempreg1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	/* 7. Configure SSM bit*/
	tempreg1 |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempreg1;
	pSPIHandle->pSPIx->CR2 = tempreg2;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
		if(pSPIx == SPI1){
			SPI1_REG_RESET();
		}
		else if(pSPIx == SPI2){
			SPI2_REG_RESET();
		}
		else if(pSPIx == SPI3){
			SPI3_REG_RESET();
		}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagname) /* This function is an aid to the send and receive functions */
{
	/*Flag names have been defined with their respective position*/
	if(pSPIx->SR & flagname) // Masking with and operator
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		/* 1. Wait until TXE (TX Empty flag) is set -  Data should be loaded only when the TX Buffer is empty, else already existing data may be lost */
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); //while( !(pSPIx->SR & (1 << 1)) );


		/* 2. Check the Data Size(DFF) in CR2 register */

		//TODO: Configure other data sizes as well

		/*CUSTOM CODE LOGIC FOR 8 AND 16 BITS DS*/
		uint16_t tempvar = pSPIx->CR2;
		tempvar = (tempvar >> SPI_CR2_DS);

		if(tempvar == SPI_DFF_16BITS) //16-bit mode
		{
			// Load the data onto the data register DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;

		}else if(tempvar == SPI_DFF_8BITS) //8-bit mode
		{
			// Load the data onto the data register DR
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;

		}

	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{


}


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{


}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{


}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{


}


void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
