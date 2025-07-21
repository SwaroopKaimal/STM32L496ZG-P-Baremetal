/*
 * stm32l496xx_uart.h
 *
 *  Created on: Jul 20, 2025
 *      Author: Swaroop Kaimal
 */

#ifndef INC_STM32L496XX_LPUART_H_
#define INC_STM32L496XX_LPUART_H_


#include "stm32l496xx.h"


typedef struct
{
	uint8_t LPUART_Mode;
	uint32_t LPUART_Baud;
	uint8_t LPUART_StopBits;
	uint8_t LPUART_WordLength;
	uint8_t LPUART_ParityControl;
	uint8_t LPUART_HWFlowControl;

}LPUART_Config_t;



typedef struct
{
	LPUART_RegDef_t *pLPUART;
	LPUART_Config_t	LPUART_Config;

}LPUART_Handel_t;



/****************************************************************************************************
 *                               APIs Supported by this LPUART Driver Code
 ****************************************************************************************************/

/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);



#endif /* INC_STM32L496XX_LPUART_H_ */
