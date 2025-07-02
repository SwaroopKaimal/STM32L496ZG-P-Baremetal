/*
 * stm32l496xx_spi.h
 *
 *  Created on: Jun 25, 2025
 *      Author: Swaroop Kaimal
 */

#ifndef INC_STM32L496XX_SPI_H_
#define INC_STM32L496XX_SPI_H_

#include "stm32l496xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
  */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * @SPI_BusConfig - Full-Duplex, Half-Duplex and Simplex
  */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
  */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


/*
 * @SPI_DFF
  */

#define SPI_DFF_4BITS		3
#define SPI_DFF_5BITS		4
#define SPI_DFF_6BITS		5
#define SPI_DFF_7BITS		6
#define SPI_DFF_8BITS		7
#define SPI_DFF_9BITS		8
#define SPI_DFF_10BITS		9
#define SPI_DFF_11BITS		10
#define SPI_DFF_12BITS		11
#define SPI_DFF_13BITS		12
#define SPI_DFF_14BITS		13
#define SPI_DFF_15BITS		14
#define SPI_DFF_16BITS		15

/*
 * @SPI_CPOL
  */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/*
 * @SPI_CPHA
  */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0


/*
 * @SPI_SSM
  */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1



/*SPI Flag related definitions: Flag NAMES, below are used for masking operation in SPI_GetFlagStatus function*/
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_CRCERR_FLAG		(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)
#define SPI_FRLVL_FLAG		(1 << SPI_SR_FRLVL)
#define SPI_FTLVL_FLAG		(1 << SPI_SR_FTLVL)



/****************************************************************************************************
 *                               APIs Supported by this SPI Driver Code
 ****************************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Polling based Send Receive APIs*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len); //Use length as uint32_t - Standard Practice
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



/*Other Peripheral COntrol APIs*/
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


#endif /* INC_STM32L496XX_SPI_H_ */
