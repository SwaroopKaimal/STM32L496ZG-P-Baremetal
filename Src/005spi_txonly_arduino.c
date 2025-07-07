/*
 * 005spi_txonly_arduino.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Swaroop Kaimal
 */

/* Using SPI 1 for this application
 *
 * PB3 -  SPI1_SCK (Alternate Function 5),
 * PB4-  SPI1_MISO (Alternate Function 5),
 * PB5- SPI1_MOSI (Alternate Function 5),
 * PB0- SPI1_NSS (Alternate Function 5),
 *
 * PB7- SPI1_NSS GPIO Output (Use if automatic NSS=0 doesn't happen)
 *
 * USE other GPIO pin for NSS if it's not pulled low automatically
	gpio_nss.pGPIOx = GPIOB;
	gpio_nss.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_nss.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_nss.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_nss.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_PP;
	gpio_nss.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&gpio_nss);
*/



#include <string.h>
#include "stm32l496xx.h"

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

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4; //MISO
	//GPIO_Init(&SPIPins);

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
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //Generated 2MHz
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

int main(void){

	char user_data[] = "Hello World";

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

		//GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, LOW);

		/*First send length of the data to be sent*/
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI1, &dataLen, 1); //Arduino code expects one byte of data length information

		SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

		/*Confirm that SPI is not busy before disabling, full data may not be sent if closed abruptly*/
		while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

		SPI_Peripheral_Control(SPI1, DISABLE);

		//GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, HIGH);

	}

	return 0;
}
