/*
 * stm32f446xx_driver_usart.h
 *
 *  Created on: 12-Jan-2023
 *      Author: dsour
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t  USART_Mode;			//!<Possible values @USART_Mode
	uint32_t USART_Baud;			//!<Possible values @USART_Baud
	uint8_t  USART_NoOf_StopBits;	//!<Possible values @USART_NoOf_StopBits
	uint8_t	 USART_Word_Len;		//!<Possible values @USART_Word_Len
	uint8_t  USART_Parity_ctrl;		//!<Possible values @USART_Parity_ctrl
	uint8_t  USART_HW_Flow_ctrl;	//!<Possible values @USART_HW_Flow_ctrl
}USART_Config_t;

/*
 * Handle structure of USARTx peripheral
 */
typedef struct
{
	USART_Reg_t		*pUSARTx;			//Base address
	USART_Config_t	USART_Config; 	//For configuration
}USART_Handle_t;


////////MACROS for controlling USART/////////
/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX			0
#define USART_MODE_ONLY_RX			1
#define USART_MODE_TXRX				2


/*
 * @USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 * @USART_NoOf_StopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 * @USART_Word_Len
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 * @USART_Parity_ctrl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 * @USART_HW_Flow_ctrl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART status flag
 */

#define USART_FLAG_PE					(1 << USART_SR_PE)
#define USART_FLAG_FE					(1 << USART_SR_FE)
#define USART_FLAG_NE					(1 << USART_SR_NE)
#define USART_FLAG_ORE					(1 << USART_SR_ORE)
#define USART_FLAG_IDLE					(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE					(1 << USART_SR_RXNE)
#define USART_FLAG_TC					(1 << USART_SR_TC)
#define USART_FLAG_TXE					(1 << USART_SR_TXE)
#define USART_FLAG_LBD					(1 << USART_SR_LBD)
#define USART_FLAG_CTS					(1 << USART_SR_CTS)


/****************************************
 * 			Supported APIs
 ***************************************/

/*
 * Peripheral Clock setup
 */
void 	 USART_PCLK(USART_Reg_t *pUSARTx, uint8_t EnDi);

/*
 * Init & DeInit
 */
void 	 USART_Init(USART_Handle_t *pUSARTHandle);
void 	 USART_DeInit(USART_Reg_t *pUSARTx);

/*
 * Data send and receive
 */
void 	 USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void 	 USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t  USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t  USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);


/*
 * IRQ config & ISR handling
 */
void	 USART_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void 	 USART_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);
void 	 USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */
uint8_t  USART_GetFlagStatus(USART_Reg_t *pUSARTx, uint32_t FlagName);
void 	 USART_Peripheral_Control(USART_Reg_t *pUSARTx, uint8_t EnDi);
void     USART_ClearFlag(USART_Reg_t *pUSARTx, uint16_t StatusFlagName);
void 	 USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
