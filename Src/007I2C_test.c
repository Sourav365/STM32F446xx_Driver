/*
 * 007I2C_test.c
 *
 *  Created on: 10-Jan-2023
 *      Author: dsour
 */

#include <stdio.h>
#include <string.h>
#include "../drivers/Inc/stm32f446xx.h"

#define MY_ADDRESS  0x42
#define SLAVE_ADDR  0x68

/*
 * I2C-1:
 * SCL->PB6
 * SDA->PB7
 */
I2C_Handle_t I2C1Handle;

void I2C1_GPIOInit()
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNET;
	I2CPins.GPIO_PinConfig.GPIO_PinOutType = GPIO_OP_TYPE_OPENDRAIN;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPDCTRL_PULLUP;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);

}
void I2C1_Init()
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACK_Control = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_Device_Address = 0x42;
	I2C1Handle.I2C_Config.I2C_FM_DutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCL_Speed = I2C_SCL_SPEED_NORMAL;

	I2C_Init(&I2C1Handle);
}

int main()
{
	char user_data[] = "Hello World";

	//I2C Pin initialize
	I2C1_GPIOInit();

	//I2C Peripheral initialize
	I2C1_Init();

	//I2C1 Enable
	I2C_Peripheral_Control(I2C1, ENABLE);

	//Send data to slave
	I2C_MasterSendData(&I2C1Handle, user_data, strlen((char*)user_data), SLAVE_ADDR);
	return 0;
}
