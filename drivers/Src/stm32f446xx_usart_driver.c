/*
 * stm32f446xx_driver_usart.c
 *
 *  Created on: 12-Jan-2023
 *      Author: dsour
 */

#include "../Inc/stm32f446xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */
void USART_PCLK(USART_Reg_t *pUSARTx, uint8_t EnDi)
{
	if(EnDi){	//EnDi == ENABLE (1)
		if	   (pUSARTx == USART1)		USART1_PCLK_EN();
		else if(pUSARTx == USART2)		USART2_PCLK_EN();
		else if(pUSARTx == USART3)		USART3_PCLK_EN();
		else if(pUSARTx == UART4)		UART4_PCLK_EN();
		else if(pUSARTx == UART5)		UART5_PCLK_EN();
		else if(pUSARTx == USART6)		USART6_PCLK_EN();
		}
		else{
		if	   (pUSARTx == USART1)		USART1_PCLK_DI();
		else if(pUSARTx == USART2)		USART2_PCLK_DI();
		else if(pUSARTx == USART3)		USART3_PCLK_DI();
		else if(pUSARTx == UART4)		UART4_PCLK_DI();
		else if(pUSARTx == UART5)		UART5_PCLK_EN();
		else if(pUSARTx == USART6)		USART6_PCLK_EN();
		}
}

/*
 * Init & DeInit
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg=0;

	//0. Enable peripheral clock
	USART_PCLK(pUSARTHandle->pUSARTx, ENABLE);

	/*******************Configure CR1 reg*******************/
	//1. Enable USART Tx & Rx according to USART mode
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
		tempreg |= (1 << USART_CR1_RE);
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
		tempreg |= (1 << USART_CR1_TE);
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
		tempreg |= (3 << USART_CR1_RE);

	//2. Code to config word length
	tempreg |= pUSARTHandle->USART_Config.USART_Word_Len << 12;

	//3. Configure parity control bit flag
	if(pUSARTHandle->USART_Config.USART_Parity_ctrl == USART_PARITY_EN_EVEN)
	{
		//Enable parity control en
		tempreg |= (1 << USART_CR1_PCE);
		//parity selection = 0 (default)
	}else if(pUSARTHandle->USART_Config.USART_Parity_ctrl == USART_PARITY_EN_ODD)
	{
		//Enable parity control en
		tempreg |= (1 << USART_CR1_PCE);
		//parity selection = 1 (odd parity)
		tempreg |= (1 << USART_CR1_PS);
	}
	//Write to CR1 reg
	pUSARTHandle->pUSARTx->CR1 = tempreg;


	/*******************Configure CR2 reg*******************/
	tempreg = 0;
	//3. config number of stop bits
	tempreg |= pUSARTHandle->USART_Config.USART_NoOf_StopBits << USART_CR2_STOP;

	//Write to CR2 reg
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/*******************Configure CR3 reg*******************/
	tempreg = 0;

	//4. config of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HW_Flow_ctrl == USART_HW_FLOW_CTRL_CTS)
		tempreg |= (1 << USART_CR3_CTSE);
	else if(pUSARTHandle->USART_Config.USART_HW_Flow_ctrl == USART_HW_FLOW_CTRL_RTS)
		tempreg |= (1 << USART_CR3_RTSE);
	else if(pUSARTHandle->USART_Config.USART_HW_Flow_ctrl == USART_HW_FLOW_CTRL_CTS_RTS)
		tempreg |= (3 << USART_CR3_RTSE);

	//Write to CR3 reg
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/*******************Configure BRR reg*******************/
	tempreg = 0;
///////////
////////
//////
////
//
}


void USART_DeInit(USART_Reg_t *pUSARTx)
{
	if     (pUSARTx == USART1)		USART1_REG_RESET();
	else if(pUSARTx == USART2)		USART2_REG_RESET();
	else if(pUSARTx == USART3)		USART3_REG_RESET();
	else if(pUSARTx == UART4)		UART4_REG_RESET();
	else if(pUSARTx == UART5)		UART5_REG_RESET();
	else if(pUSARTx == USART6)		USART6_REG_RESET();
}

/*
 * Data send and receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pdata;

	//Until all bits not transmitted
	for(uint32_t i=0; i < len; i++)
	{
		//Wait until TXE flag is set
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		//Check word length
		if(pUSARTHandle->USART_Config.USART_Word_Len == USART_WORDLEN_9BITS)
		{
			//load DR with 2 bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF); //0000_0001__1111_1111

			//Check USART parity control
			if(pUSARTHandle->USART_Config.USART_Parity_ctrl == USART_PARITY_DISABLE)
			{
				//When no parity, 9-bit user data will be sent
				//increase pTxBuffer 2 times, 2-bytes of data (9-bit)
				pTxBuffer += 2;
			}else
			{
				//8-bit user data+1 bit parity, 9th bit will be replaced by parity bit by software
				pTxBuffer++;
			}
		}else //8-bit data transfer
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF); //Take last 8 bit data
			pTxBuffer++;
		}
	}
	//Wait till TC flag is set in the SR
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	//until all bits are not transmitted
	for(uint32_t i = 0 ; i < len; i++)
	{
		//wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_Word_Len == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_Parity_ctrl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data
				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer+=2;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//receive 8bit data in a frame
			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_Parity_ctrl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txstate = pUSARTHandle->TODO;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TODO = Len;
		pUSARTHandle->pTxBuffer = TODO;
		pUSARTHandle->TxBusyState = TODO;

		//Implement the code to enable interrupt for TXE
		TODO


		//Implement the code to enable interrupt for TC
		TODO


	}

	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t rxstate = pUSARTHandle->TODO;

	if(rxstate != TODO)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = TODO;
		pUSARTHandle->RxBusyState = TODO;

		//Implement the code to enable interrupt for RXNE
		TODO

	}

	return rxstate;

}
*/

/*
 * IRQ config & ISR handling
 */
void USART_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void USART_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */
uint8_t USART_GetFlagStatus(USART_Reg_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName) return FLAG_SET;
	else					    return FLAG_RESET;
}

void USART_Peripheral_Control(USART_Reg_t *pUSARTx, uint8_t EnDi)
{
	if(EnDi == ENABLE)	pUSARTx->CR1 |=  (1 << USART_CR1_UE);
	else				pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
}
void USART_ClearFlag(USART_Reg_t *pUSARTx, uint16_t StatusFlagName);


void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << TODO))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   TODO
  }

  //Calculate the Mantissa part
  M_part = TODO/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (TODO * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * TODO)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * TODO)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->TODO = tempreg;
}

/*
 * Application Callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent);

