/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 22-Dec-2022
 *      Author: dsour
 */


#include "../Inc/stm32f446xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */
/*****************************************************************
 * @function				-> SPI_PCLK
 *
 * @brief					-> Enable/Disable clk of a given SPI port
 *
 * @param1					-> Base address of SPI peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_PCLK(SPI_Reg_t *pSPIx, uint8_t EnDi)
{
	if(EnDi){	//EnDi == ENABLE (1)
			if	   (pSPIx == SPI1)		SPI1_PCLK_EN();
			else if(pSPIx == SPI2)		SPI2_PCLK_EN();
			else if(pSPIx == SPI3)		SPI3_PCLK_EN();
			else if(pSPIx == SPI4)		SPI4_PCLK_EN();
		}
		else{
			if	   (pSPIx == SPI1)		SPI1_PCLK_DI();
			else if(pSPIx == SPI2)		SPI2_PCLK_DI();
			else if(pSPIx == SPI3)		SPI3_PCLK_DI();
			else if(pSPIx == SPI4)		SPI4_PCLK_DI();
		}
}


/*
 * Init & DeInit
 */
/*****************************************************************
 * @function				-> SPI_Init
 *
 * @brief					-> Initialize a given SPI peripheral
 *
 * @param1					-> SPI handler
 * @param2					->
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	//0. Enable SPI peripheral clk
	SPI_PCLK(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;

	//1. Config device moede
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode) << SPI_CR1_MSTR;

	//2. Config bus config
	switch (pSPIHandle->SPI_Config.SPI_Bus_Config){
		case SPI_BUS_CONFIG_FULL_DUPLEX:
		{
			//Bi-directional mode cleared-> unidirectional->MOSI+MISO
			tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		}
		case SPI_BUS_CONFIG_HALF_DUPLEX:
		{
			//Bi-directional mode set
			tempreg |= (1 << SPI_CR1_BIDI_MODE);
		}
		case SPI_BUS_CONFIG_SIMPLEX_RX:
		{
			//Bi-directional mode cleared
			tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
			//RX-only bit must be set
			tempreg |= (1 << SPI_CR1_RXONLY);
		}
	}

	//3. Config SPI serial clk speed (baud rate)
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed) << SPI_CR1_BR;

	//4.  Config DFF
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF) << SPI_CR1_DFF;

	//5. Config CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL) << SPI_CR1_CPOL;

	//6. Config CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA) << SPI_CR1_CPHA;

	//7. Config SSM
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM) << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*****************************************************************
 * @function				-> SPI_DeInit
 *
 * @brief					-> Disable given SPI peripheral
 *
 * @param1					-> Base address of SPI peripheral
 * @param2					->
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_DeInit(SPI_Reg_t *pSPIx)
{
	if     (pSPIx == SPI1)		SPI1_REG_RESET();
	else if(pSPIx == SPI2)		SPI2_REG_RESET();
	else if(pSPIx == SPI3)		SPI3_REG_RESET();
	else if(pSPIx == SPI4)		SPI4_REG_RESET();
}


/*****************************************************************
 * @function				-> SPI_GetFlagStatus
 *
 * @brief					-> Get status of flags from SPI Status reg.
 *
 * @param1					-> Base address of SPI peripheral
 * @param2					-> Flag name
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
uint8_t SPI_GetFlagStatus(SPI_Reg_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName) return FLAG_SET;
	else					  return FLAG_RESET;
}

/*
 * Data send and receive
 */
/*****************************************************************
 * @function				-> SPI_SendData
 *
 * @brief					-> Send data until all data sent
 *
 * @param1					-> Base address of SPI peripheral
 * @param2					-> Pointer of stored data
 * @param3					-> Length of data
 *
 * @return					-> none
 *
 * @note					-> It is a blocking function
 *
 */
void SPI_SendData(SPI_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set (when content of TX-buffer = 0, TXE=1
		//while(!((pSPIx->SR) & (1<<SPI_SR_TXE)));
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check DFF bit in CR1
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16-bit DFF
			//i) Load data to DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);	//a)convert to 16 bit pointer b)retrieve data from 16 bit address
			Len = Len-2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8-bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*****************************************************************
 * @function				-> SPI_ReceiveData
 *
 * @brief					-> Receive data until all content of RX received
 *
 * @param1					-> Base address of SPI peripheral
 * @param2					-> Pointer of stored data
 * @param3					-> Length of data
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_ReceiveData(SPI_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set (when content of RX-buffer = FULL, RXNE=1
		//while(!((pSPIx->SR) & (1<<SPI_SR_RXNE)));
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check DFF bit in CR1
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16-bit DFF
			//i) Load data from DR to Rx-buffer
			*((uint16_t*) pRxBuffer) = pSPIx->DR;	//a)convert to 16 bit pointer b)retrieve data from 16 bit address
			Len = Len-2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8-bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * Other peripheral control APIs
 */
/*****************************************************************
 * @function				-> SPI_Peripheral_Control
 *
 * @brief					-> Enable/Disable SPI Peripheral
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_Peripheral_Control(SPI_Reg_t *pSPIx, uint8_t EnDi)
{
	if(EnDi == ENABLE)	pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	else				pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

/*****************************************************************
 * @function				-> SPI_SSIConfig
 *
 * @brief					-> Enable/Disable SSI bit
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_SSIConfig(SPI_Reg_t *pSPIx, uint8_t EnDi)
 {
	 if(EnDi == ENABLE)	pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	 else				pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
 }

/*****************************************************************
 * @function				-> SPI_SSOEConfig
 *
 * @brief					-> Enable/Disable SSOE bit
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_SSOEConfig(SPI_Reg_t *pSPIx, uint8_t EnDi)
{
	 if(EnDi == ENABLE)	pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	 else				pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
}

/*
 * IRQ config & ISR handling
 */
/*****************************************************************
 * @function				-> GPIO_PCLK
 *
 * @brief					-> Enable/Disable clk of a given GPIO port
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);

/*****************************************************************
 * @function				-> GPIO_PCLK
 *
 * @brief					-> Enable/Disable clk of a given GPIO port
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);

/*****************************************************************
 * @function				-> GPIO_PCLK
 *
 * @brief					-> Enable/Disable clk of a given GPIO port
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * IRQ config & ISR handling
 */



