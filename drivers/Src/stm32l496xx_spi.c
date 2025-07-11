/*
 * stm32l496xx_spi.c
 *
 *  Created on: Jun 25, 2025
 *      Author: Swaroop S Kaimal
 */

#include "stm32l496xx_spi.h"

/*Helper functions for SPI interrupt handler, so the prototypes are not declared in the header file.
 * Use 'static' keyword to indicate that its a private function. */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
		/*The FRXTH bit that will be set for 8 bit SPI communication will be present
		 * in the CR2 register, so make sure that we take only the last 4 bits*/
		tempvar &= 0x0F;


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
			*(volatile uint8_t *)&pSPIx->DR = *(pTxBuffer); /* Refer http://efton.sk/STM32/gotcha/g22.html */
			len--;
			pTxBuffer++;

			/*
			 * The SPIx_DR register is not a normal memory location where writing and reading access the
			 * same storage. Instead, writing loads the output shift register, while reading reads the
			 * received input. Depending on device details, reading may also "claim" the input, clearing
			 * it from the register until another word is received. For these reasons, trying to watch
			 * the SPI DR with a debugger is not only not going to give you the information you seek, it
			 * may even be damaging to the data you would otherwise receive.
			 */

		}

	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len > 0)
	{
		/* 1. Wait until RXNE (TX Empty flag) is set*/
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);


		/* 2. Check the Data Size(DFF) in CR2 register */

		//TODO: Configure other data sizes as well

		/*CUSTOM CODE LOGIC FOR 8 AND 16 BITS DS*/
		uint16_t tempvar = pSPIx->CR2;
		tempvar = (tempvar >> SPI_CR2_DS);
		tempvar &= 0x0F; /* To get the last 4 bits, see logic in send data function*/

		if(tempvar == SPI_DFF_16BITS) //16-bit mode
		{
			// Load the data from DR RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRxBuffer++;

		}else if(tempvar == SPI_DFF_8BITS) //8-bit mode
		{

			// Load the data from DR to the buffer
			*(pRxBuffer) = pSPIx->DR;
			len--;
			pRxBuffer++;

		}

	}

}


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
			/*Program ISER2 Register - L496ZG-P has only 90 IRQs implemented so only 3 are well enough*/
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
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



void SPI_FRXTHConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_FRXTH);
		}else{
			pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH);
		}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	/*In IT function, we are not writing data directly into the data register, that will be done in Interrupt Handler.
	 * This function just saves the pointers, state and length information, enable the TXEIE interrupt and returns.
	 *
	 * So a place holder has to be created in order to save the application's TX address, length and SPI state. That
	 * is done by modifying the SPI_Handle_t structure and and adding these parameters:
	 *
	 * 	uint8_t *pTxBuffer;
	 *	uint8_t *pRxBuffer;
	 *	uint32_t TxLen;
	 *	uint32_t RxLen;
	 *	uint8_t TxState;
	 *	uint8_t RxState;
	 * */

	/*1. Check if SPI is already busy*/
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)

	{
		/* 2. Save the Tx-buffer address and length information in some global variable*/
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		/* 3. Mark the SPI state as busy in transmission so that no other code can take over the same SPI
		 * till the transmission is over*/
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/* 4. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR*/
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	/*1. Check if SPI is already busy*/
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)

	{
		/* 2. Save the Tx-buffer address and length information in some global variable*/
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		/* 3. Mark the SPI state as busy in transmission so that no other code can take over the same SPI
		 * till the transmission is over*/
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* 4. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR*/
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}

	return state;

}



void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	/*1. Check for the TXE*/
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE)) >> SPI_SR_TXE;
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE)) >> SPI_CR2_TXEIE;

	if(temp1 && temp2)
	{
		/*Handle TXE*/
		spi_txe_interrupt_handle(pSPIHandle); //Helper function
	}

	/*2. Check for the RXNE*/
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		/*Handle RXNE*/
		spi_rxne_interrupt_handle(pSPIHandle); //Helper function
	}

	/*3. Check for the Overrun error (OVR) flag*/
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		/*Handle OVR error*/
		spi_ovr_err_interrupt_handle(pSPIHandle); //Helper function
	}
}

/*Helper functions*/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	/*Similar logic as SPI_SendData() polling function (errors solved)*/
	uint16_t tempvar = pSPIHandle->pSPIx->CR2;
	tempvar = (tempvar >> SPI_CR2_DS);
	/*The FRXTH bit that will be set for 8 bit SPI communication will be present
	 * in the CR2 register, so make sure that we take only the last 4 bits*/
	tempvar &= 0x0F;

	/*Get length information from the SPI_Handle_t*/
	uint32_t len = pSPIHandle->TxLen;

	if(tempvar == SPI_DFF_16BITS) //16-bit mode
	{
		// Load the data onto the data register DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		len--;
		len--;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	}else if(tempvar == SPI_DFF_8BITS) //8-bit mode
	{
		// Load the data onto the data register DR
		*(volatile uint8_t *)&pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer); /* Refer http://efton.sk/STM32/gotcha/g22.html */
		len--;
		pSPIHandle->pTxBuffer++;
	}

	if(len == 0)
	{
		//When length is zero close the SPI communication and inform the  application

		SPI_CloseTransmission(pSPIHandle);

		/*Call back to inform the application*/
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	/*Similar logic as SPI_ReceiveData() polling function (errors solved)*/
	uint16_t tempvar = pSPIHandle->pSPIx->CR2;
	tempvar = (tempvar >> SPI_CR2_DS);
	/*The FRXTH bit that will be set for 8 bit SPI communication will be present
	 * in the CR2 register, so make sure that we take only the last 4 bits*/
	tempvar &= 0x0F;

	/*Get length information from the SPI_Handle_t*/
	uint32_t len = pSPIHandle->RxLen;

	if(tempvar == SPI_DFF_16BITS) //16-bit mode
	{
		// Load the data onto the data register DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		len--;
		len--;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else if(tempvar == SPI_DFF_8BITS) //8-bit mode
	{
		// Load the data onto the data register DR
		*(pSPIHandle->pRxBuffer) = *(volatile uint8_t *)&pSPIHandle->pSPIx->DR; /* Refer http://efton.sk/STM32/gotcha/g22.html */
		len--;
		pSPIHandle->pRxBuffer++;
	}

	if(len == 0)
	{

        //When length is zero close the SPI communication and inform the  application

		SPI_CloseReception(pSPIHandle);

		/*Call back to inform the application*/
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	/* 1. Clear the OVR flag - RM:  Clearing the OVR bit is done by a read access to the SPI_DR register
	 * followed by a read access to the SPI_SR register.*/

	/*If the SPI peripheral is busy in transmission, if then OVR error happens, flag wont be reset to prevent
	 * anomalies, the application will be notified. When the application receives the SPI_EVENT_OVR_ERR event, it has
	 * to call the SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) explicitly.*/

	uint8_t temp;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void)temp; /*To remove the unused temp variable warning*/

	/*2. Infrom the application*/
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	/*Deactivate the TXEIE bit to prevent further interrupts from setting of TXE flag*/
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );

	/*Reset Tx-buffer, length and state*/
	pSPIHandle->pTxBuffer = NULL; //NULL in stddef.h
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	/*Deactivate the RXNEIE bit to prevent further interrupts from setting of RXE flag*/
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );

	/*Reset Tx-buffer, length and state*/
	pSPIHandle->pRxBuffer = NULL; //NULL in stddef.h
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	/* 1 Clear the OVR flag - RM:  Clearing the OVR bit is done by a read access to the SPI_DR register
	* followed by a read access to the SPI_SR register.*/
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp; /*To remove the unused temp variable warning*/
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	/*This has to be implemented by the application, but we do not know whether the user needs to implement this in their code.
	 * If this is not implemented the compiler will issue errors. Hence, we have to use 'weak' GCC attribute.*/

	/*The user application may override this function*/
}

