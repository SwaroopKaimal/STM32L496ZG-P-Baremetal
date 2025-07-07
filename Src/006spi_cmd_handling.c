/*
 * 006spi_cmd_handling.c
 *
 *  Created on: Jul 4, 2025
 *      Author: Swaroop Kaimal

 * Using SPI 1 for this application
 *
 * PB3 -  SPI1_SCK (Alternate Function 5),
 * PB4-  SPI1_MISO (Alternate Function 5),
 * PB5- SPI1_MOSI (Alternate Function 5),
 * PB0- SPI1_NSS (Alternate Function 5),

*/
#include <string.h>
#include "stm32l496xx.h"

/*Command codes*/
#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_ON		1
#define LED_OFF 	0

/*Arduino analog pins*/
#define ANALOG_PIN_0 	0
#define ANALOG_PIN_1 	1
#define ANALOG_PIN_2	2
#define ANALOG_PIN_3 	3
#define ANALOG_PIN_4 	4
#define ANALOG_PIN_5 	5

/*Arduino LED Pin*/
#define LED_PIN 	9


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
	SPI_Handle_t SPI1handle;

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

void GPIO_Button_Init()
{
	GPIO_Handle_t gpio_btn;


	gpio_btn.pGPIOx = GPIOC;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP; //Applicable only when the mode is output
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //(Check for L496ZG - Pulled Down for PC13) Usuallu External pull up already available in Nucleo Board (Pull down in Discovery)

	GPIO_Init(&gpio_btn);


}

void delay(){
	for(uint32_t i=0; i<250000; i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5) //According to the client code
	{
		//ACK
		return 1;
	}
	return 0;
}

int main(void){

	/*SPI won't initiate the communication by itself to receive the ACK or NACK byte, so we have to send a dummy variable*/
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read; /*When we read the  Data Register, RXNE flag will be automatically cleared.*/

	GPIO_Button_Init();

	SPI1_GPIOInit();

	SPI1_Init();

	/*SS output is enabled in master mode and when the SPI interface is enabled in hardware mode.*/
	SPI_SSOEConfig(SPI1, ENABLE);

	while(1){

		while(!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));

		delay();

		SPI_FRXTHConfig(SPI1, ENABLE); /* Specific to L4 series, check SPI CR2 register*/

		/*It recommended to configure all SPI parameters while its not active, when its active it will
		 * be performing communications continuously and will not accept any changes. So we have the set
		 * the SPE bit in CR1 at this stage*/
		SPI_Peripheral_Control(SPI1, ENABLE);



		/*--------------- 1. CMD_LED_CTRL <pin no (1)>  <value(1)> ---------------*/

		uint8_t commandcode = CMD_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];


		SPI_SendData(SPI1, &commandcode, 1);

		/*In SPI when master or slave sends 1 byte of data, it also receives 1 byte of data. This
		 * transmission of 1 byte results in the collection of 1 garbage byte in Rx buffer of the
		 * master and RXNE flag is set. So do the dummy read and clear the RXNE flag. When we read
		 * the  Data Register, RXNE flag will be automatically cleared.
		 *
		 * Reading the SPIx_DR register differs from reading a regular variable. The hardware is
		 * specifically designed to detect when read accesses are made to the SPIx_DR register and
		 *  respond accordingly by clearing the RXNE flag.*/
		SPI_ReceiveData(SPI1, &dummy_read, 1);

		//Send dummy byte to fetch the response from the slave
		SPI_SendData(SPI1, &dummy_write, 1);

		SPI_ReceiveData(SPI1, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			/*Send arguments if the response is ACK*/
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI1, args, 2);
		}

		/*--------------- 2. CMD_SENSOR_READ <analog pin number(1)> ---------------*/

		while(!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));

		delay();

		commandcode = CMD_SENSOR_READ;

		SPI_SendData(SPI1, &commandcode, 1);

		SPI_ReceiveData(SPI1, &dummy_read, 1); //To clear RXNE

		//Send dummy byte to fetch the response from the slave
		SPI_SendData(SPI1, &dummy_write, 1);

		SPI_ReceiveData(SPI1, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			/*Send arguments if the response is ACK*/
			args[0] = ANALOG_PIN_0;
			SPI_SendData(SPI1, args, 2);

			/*Slave actually takes some time to read the analog value from the ADC, so master should wait
			 * for some time before generating the dummy bits to fetch the result, so add delay*/
			delay();

			//Send dummy byte to fetch the response from the slave
			SPI_SendData(SPI1, &dummy_write, 1);
			uint8_t analog_read;
			SPI_ReceiveData(SPI1, &analog_read, 1);
		}

		SPI_ReceiveData(SPI1, &dummy_read, 1); //To clear RXNE

		/*Confirm that SPI is not busy before disabling, full data may not be sent if closed abruptly*/
		while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));


		SPI_Peripheral_Control(SPI1, DISABLE);

	}

	return 0;
}


