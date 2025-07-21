/*
 * stm32l496xx_i2c.h
 *
 *  Created on: Jul 12, 2025
 *      Author: Swaroop Kaimal
 */

#ifndef INC_STM32L496XX_I2C_H_
#define INC_STM32L496XX_I2C_H_


#include "stm32l496xx.h"

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress; /*Device address will be input by the user*/
	//uint8_t  I2C_ACKControl; //Not needed in L4, Enabled by default
	//uint16_t I2C_FMDutyCycle; //No duty bit in L4

}I2C_Config_t;


typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		1
#define I2C_SCL_SPEED_FM		2


/*
 * #I2C_ACKControl
 */
#define I2C_ACK_ENABLE			0
#define I2C_ACK_DISABLE			1


/*
 * #I2C_FMDutyCycle ----------------- ??
 */
#define I2C_FM_DUTY_2
#define I2C_FM_DUTY_16_9

/*I2C Flag related definitions: Flag NAMES, below are used for masking operation in SPI_GetFlagStatus function*/
#define I2C_TXE_FLAG			(1 << I2C_ISR_TXE)
#define I2C_TXIS_FLAG			(1 << I2C_ISR_TXIS)
#define I2C_RXNE_FLAG			(1 << I2C_ISR_RXNE)
#define I2C_ADDR_FLAG			(1 << I2C_ISR_ADDR)
#define I2C_NACKF_FLAG			(1 << I2C_ISR_NACKF)
#define I2C_STOPF_FLAG			(1 << I2C_ISR_STOPF)
#define I2C_TC_FLAG				(1 << I2C_ISR_TC)
#define I2C_TCR_FLAG			(1 << I2C_ISR_TCR)
#define I2C_BERR_FLAG			(1 << I2C_ISR_BERR)
#define I2C_ARLO_FLAG			(1 << I2C_ISR_ARLO)
#define I2C_OVR_FLAG			(1 << I2C_ISR_OVR)
#define I2C_PECERR_FLAG			(1 << I2C_ISR_PECERR)
#define I2C_TIMEOUT_FLAG		(1 << I2C_ISR_TIMEOUT)
#define I2C_ALERT_FLAG			(1 << I2C_ISR_ALERT)
#define I2C_BUSY_FLAG			(1 << I2C_ISR_BUSY)
#define I2C_DIR_FLAG			(1 << I2C_ISR_TXE)


//Other Macros
#define I2C_DISABLE_SR 			0
#define I2C_ENABLE_SR 			1

/****************************************************************************************************
 *                               APIs Supported by this I2C Driver Code
 ****************************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*Polling based Send Receive APIs*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);


/*Interrupt based Send Receive APIs*/


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);




/*Other Peripheral Control APIs*/
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagname);



/*Application callback*/
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
/*This has to be implemented by the application, but we do not know whether the user needs to implement this in their code.
 * If this is not implemented the compiler will issue errors. Hence, we have to use 'weak' keyword.*/




#endif /* INC_STM32L496XX_I2C_H_ */
