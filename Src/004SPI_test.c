/*
 * 004SPI_test.c
 *
 *  Created on: 24-Dec-2022
 *      Author: dsour
 */


#include "../drivers/Inc/stm32f446xx.h"
#include <string.h>

/*
 * Specifications
 * 		->mode SPI2- MASTER
 * 		->SCLK- Max possible
 * 		->i) DFF=0, ii) DFF=1
 */

/*
 * MOSI-> PB15
 * MISO-> PB14
 * SCLK-> PB13
 * NSS -> PB12
 * Alternate fn mode-> 5
 */

void SPI2_gpioInit()
{
	GPIO_Handle_t SPPIPin;

	SPPIPin.pGPIOx = GPIOB;
	SPPIPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNET;
	SPPIPin.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPPIPin.GPIO_PinConfig.GPIO_PinOutType = GPIO_OP_TYPE_PUSHPULL;
	SPPIPin.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPDCTRL_NO;
	SPPIPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_LOW;

	//MOSI -> PB15
	SPPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPPIPin);

	//MISO -> PB14
//	SPPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&SPPIPin);

	//SCLK -> PB13
	SPPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPPIPin);

	//NSS -> PB12
//	SPPIPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
//	GPIO_Init(&SPPIPin);
}

void SPI2_Init()
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_Bus_Config = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_1ST_EDGE;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;

	SPI_Init(&SPI2Handle);
}
int main()
{
	char user_data[] = "Hello World";
	SPI2_gpioInit();
	SPI2_Init();

	//Make NSS signal HIGH.
	SPI_SSIConfig(SPI2, ENABLE);
	//Enable SPI2_peripheral
	SPI_Peripheral_Control(SPI2, ENABLE);
	//Send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//Check if SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));
	//Disable SPI2
	SPI_Peripheral_Control(SPI2, DISABLE);
	while(1);
	return 0;
}
