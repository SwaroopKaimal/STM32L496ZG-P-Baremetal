/*
 * 004spi_tx_testing.c
 *
 *  Created on: Jun 26, 2025
 *      Author: Swaroop Kaimal
 */

/* Using SPI 1 for this application
 *
 * PB3 -  SPI1_SCK (Alternate Function 5),
 * PB4-  SPI1_MISO (Alternate Function 5),
 * PB5- SPI1_MOSI (Alternate Function 5),
 * PB0- SPI1_NSS (Alternate Function 5)
 *
 * */

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

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0; //NSS
	//GPIO_Init(&SPIPins);

}

void SPI1_Init()
{
	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI1handle);

}

int main(void){

	char user_data[] = "Hello World";

	SPI1_GPIOInit();

	SPI1_Init();

	/*See video 150 - This makes NSS signal internally high and avoids MODF error and MTSR register resetting,
	 * in software slave management mode*/
	SPI_SSIConfig(SPI1, ENABLE);


	/*It recommended to configure all SPI parameters while its not active, when its active it will
	 * be performing communications continuously and will not accept any changes. So we have the set
	 * the SPE bit in CR1 at this stage*/
	SPI_Peripheral_Control(SPI1, ENABLE);

	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	/*Confirm that SPI is not busy before disabling, full data may not be sent if closed abruptly*/
	while(SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));

	SPI_Peripheral_Control(SPI1, DISABLE);

	while(1);


	return 0;
}
