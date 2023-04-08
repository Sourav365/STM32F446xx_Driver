/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 22-Dec-2022
 *      Author: dsour
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;			//!<Possible values @SPI_DeviceMode
	uint8_t SPI_Bus_Config;			//!<Possible values @SPI_Bus_Config
	uint8_t SPI_SclkSpeed;			//!<Possible values @SPI_SclkSpeed
	uint8_t SPI_DFF;				//!<Possible values @SPI_DFF
	uint8_t SPI_CPOL;				//!<Possible values @SPI_CPOL
	uint8_t SPI_CPHA;				//!<Possible values @SPI_CPHA
	uint8_t SPI_SSM;				//!<Possible values @SPI_SSM
}SPI_Config_t;

/*
 * Handle structure of SPIx peripheral
 */
typedef struct
{
	SPI_Reg_t		*pSPIx;			//Base address
	SPI_Config_t	SPI_Config; 	//For configuration
}SPI_Handle_t;


////////MACROS for controlling SPI/////////
/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * @SPI_Bus_Config
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX		1
#define SPI_BUS_CONFIG_HALF_DUPLEX		2
#define SPI_BUS_CONFIG_SIMPLEX_RX		3
//#define SPI_BUS_CONFIG_SIMPLEX_TX		4	//Same as FULL duplex connect MOSI line only in hardware.

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
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_1ST_EDGE				0
#define SPI_CPHA_2ND_EDGE				1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * SPI status flag
 */
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_CRC_ERR_FLAG				(1 << SPI_SR_CRC_ERR)
#define SPI_BSY_FLAG					(1 << SPI_SR_BSY)




/****************************************
 * 			Supported APIs
 ***************************************/

/*
 * Peripheral Clock setup
 */
void 	 SPI_PCLK(SPI_Reg_t *pSPIx, uint8_t EnDi);

/*
 * Init & DeInit
 */
void 	 SPI_Init(SPI_Handle_t *pSPIHandle);
void 	 SPI_DeInit(SPI_Reg_t *pSPIx);

/*
 * Data send and receive
 */
void 	 SPI_SendData(SPI_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void 	 SPI_ReceiveData(SPI_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ config & ISR handling
 */
void	 SPI_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void 	 SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);
void 	 SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */
uint8_t  SPI_GetFlagStatus(SPI_Reg_t *pSPIx, uint32_t FlagName);
void 	 SPI_Peripheral_Control(SPI_Reg_t *pSPI, uint8_t EnDi);
void 	 SPI_SSIConfig(SPI_Reg_t *pSPIx, uint8_t EnDi);
void 	 SPI_SSOEConfig(SPI_Reg_t *pSPIx, uint8_t EnDi);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
