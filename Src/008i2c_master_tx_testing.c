/*
 * 007spi_msg_rcv_it.c
 *
 *  Created on: Jul 16, 2025
 *      Author: Swaroop Kaimal
 *
 */



#include<stdio.h>
#include<string.h>
#include "stm32l496xx.h"

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C2Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";
/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void I2C2_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	/*Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx = GPIOF;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PO_TYPE_OD;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&I2CPins);


	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_Init(&I2CPins);


}

void I2C2_Inits(void)
{
	I2C2Handle.pI2Cx = I2C2;
	//I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	//I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C2Handle);

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

int main(void)
{

	GPIO_Button_Init();

	//i2c pin inits
	I2C2_GPIOInits();

	//i2c peripheral configuration
	I2C2_Inits();

	//enable the i2c peripheral
	I2C_Peripheral_Control(I2C2, ENABLE);





	while(1)
	{
		while(!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));

		delay();

		I2C_MasterSendData(&I2C2Handle,some_data,strlen((char*)some_data),SLAVE_ADDR, 0);

	}


}
