/*
 * stm32l496xx_i2c.c
 *
 *  Created on: Jul 12, 2025
 *      Author: Swaroop Kaimal
 */

#include "stm32l496xx.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx); //Helper function

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi ==  ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();

		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();

		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();

		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();

		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();

		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();

		}
	}

}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	/* Following steps are for STM32F4xx:

	   1. Configure the Mode (standard or fast)
	   2. Configure the speed of serial clock
	   3. Configure the device address (applicable in slave mode)
	   4. Enable the ACKing  - Already Enabled in L496
	   5. Configure the rise time for I2C pins, slew rate

	*/

	/* Steps for other family of micro-controllers like L4 and H7: */


	/*1. Enable Clock Control*/
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/* 1.  Configure the Own Address in OAR1 - OA1EN, OA1MODE, 7-bit addressing mode: OA1[7:1] contains the 7-bit own slave address.
	 * Bits OA1[9], OA1[8] and OA1[0] are don't care. 10-bit addressing mode: OA1[9:0] contains the 10-bit own slave address. Set
	 * OA1EN only after other configurations in the register. This driver is using only 7 bit addressing mode. */
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	pI2CHandle->pI2Cx->OAR1 |= (1 << I2C_OAR1_OA1EN);

	/* 2. Configure the TIMINGR for the SCLK frequency*/
//	if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) // 100 kHz
//	{
//		uint32_t tempreg = 0;
//		tempreg |= (0x1 << 28);  // PRESC[3:0] = 0x1
//		tempreg |= (0x4 << 20);  // SCLDEL[3:0] = 0x4
//		tempreg |= (0x2 << 16);  // SDADEL[3:0] = 0x2
//		tempreg |= (0xF << 8);   // SCLH[7:0] = 0xF
//		tempreg |= (0x13 << 0);  // SCLL[7:0] = 0x13
//
//		pI2CHandle->pI2Cx->TIMINGR = tempreg;
//	}
//	else if (pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM) // 400 kHz
//	{
//		uint32_t tempreg = 0;
//		tempreg |= (0x0 << 28);  // PRESC = 0
//		tempreg |= (0x1 << 20);  // SCLDEL = 1
//		tempreg |= (0x0 << 16);  // SDADEL = 0
//		tempreg |= (0x3 << 8);   // SCLH = 3
//		tempreg |= (0x9 << 0);   // SCLL = 9
//
//		pI2CHandle->pI2Cx->TIMINGR = tempreg;
//	}
	pI2CHandle->pI2Cx->TIMINGR = 0x00000E14;
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();

	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();

	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();

	}

}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR2 |= (1 << I2C_CR2_START);
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    pI2Cx->CR2 |= (SlaveAddr << 1);         // Slave address bit 7:1 (master mode)
    pI2Cx->CR2 &= ~(1 << I2C_CR2_RD_WRN);   // RD_WRN: write transfer direction (master mode)
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    pI2Cx->CR2 |= (SlaveAddr << 1);         // Slave address bit 7:1 (master mode)
    pI2Cx->CR2 |= (1 << I2C_CR2_RD_WRN);    // RD_WRN: read transfer direction (master mode)
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	/*Refer and compare with the I2C steps of STM32F4 boards - L4 lacks many features */

	// Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// Set autoend to 0, and length of data to be transmitted
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_AUTOEND);  //software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
	pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES);  //number of bytes to be transmitted

	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Send the data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->TXDR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// wait until TC flag is set (when NBYTES data are transferred)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ISR_TC)));

	if (Sr == I2C_DISABLE_SR)
	{
		// Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}


	// clear STOPF flag in I2C_ICR and clear the I2C_CR2 register:
	pI2CHandle->pI2Cx->ICR |= (1 << I2C_ICR_STOPCF);
	//pI2CHandle->pI2Cx->CR2 = 0x0;

}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
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


void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi ==  ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);

	}else{

		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);

	}
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagname)
{
	if(pI2Cx->ISR & flagname)
	{
		return FLAG_SET;

	}else
	{
		return FLAG_RESET;
	}
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{


}


