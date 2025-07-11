/*
 * 007spi_msg_rcv_it.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Swaroop Kaimal
 *
 *		Setup print via ITM code
 *      Enable SWV ITM data console to see the message
 *
 *      GPIO Interrupt from Arduino on pin PG6
 *
 *      Use Carriage Return in serial monitor on Arduino while sending message
 *
 */



#include <stdio.h>
#include <string.h>
#include "stm32l496xx.h"

#define MAX_LEN 500

/*Variables required in interrupt mode*/
char RcvBuff[MAX_LEN];
volatile char ReadByte;

SPI_Handle_t SPI1handle; //Have to be declared globally for interrupt mode

volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void SPI1_GPIOInit()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3; //SCLK
	GPIO_Init(&SPIPins); /* GPIO_Init operates only one pin at a time so call for every pin.*/

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; //MOSI
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4; //MISO
	GPIO_Init(&SPIPins);

	//Setup NSS manually - if working with internal setup
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0; //NSS
	GPIO_Init(&SPIPins);

}

void SPI1_Init()
{

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management

	SPI_Init(&SPI1handle);

}

/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));

	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOE;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&spiIntPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

}

int main(void)
{

	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	SPI1_GPIOInit();

	SPI1_Init();

	SPI_SSOEConfig(SPI1,ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI1,ENABLE);

	while(1)
	{
		rcvStop = 0;

		while(!dataAvailable); ////wait till data available interrupt from transmitter device(slave)

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,DISABLE); //Disable to avoid further interrupts from the slave

		SPI_Peripheral_Control(SPI1,ENABLE);


		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */

			/*These APIs basically enables the interrupt for SPI for further processes*/
			while(SPI_SendDataIT(&SPI1handle, &dummy, 1) == SPI_BUSY_IN_TX);


			while(SPI_ReceiveDataIT(&SPI1handle, &ReadByte, 1) == SPI_BUSY_IN_RX);

		}

		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG) );

		//Disable the SPI1 peripheral
		SPI_Peripheral_Control(SPI1,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

	}

	return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) //Overriding the __weak attributed function
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || ( i == MAX_LEN)){
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}

/* Slave data available interrupt handler */
void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	dataAvailable = 1;
}
