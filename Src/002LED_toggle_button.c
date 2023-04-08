/*
 * 002LED_toggle_button.c
 *
 *  Created on: Dec 20, 2022
 *      Author: dsour
 */

#include "../drivers/Inc/stm32f446xx.h"

void delay()
{
	for(uint32_t i=0; i<=500000;i++);
}


/* 0. Specifications needed
 * 		LED		--> GPIOA5
 * 		BUTTON	--> GPIOC13
 * 		-> Address	: GPIOA,C
 * 		-> pin no	: 5,13
 * 		-> pin mode	: OUTPUT,INPUT
 * 		-> out type	: PUSHPULL,PUSHPULL (default)
 * 		-> out speed: --,--
 * 		-> pupd reg : NO(default), Pullup already connected to board
 *
 * 1. enable clk for peripheral
 * 2. initiate peripheral according to specifications
 * 3. call APIs to read/write....
 */
int main()
{
	GPIO_Handle_t GpioLed, GpioButton;

	//Config for GpioLed
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;

	//Config for GpioButton
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	//GpioButton.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPDCTRL_PULLUP;

	//Enable clk
	GPIO_PCLK(GPIOA, ENABLE);
	GPIO_PCLK(GPIOC, ENABLE);

	//Initiate peripheral
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1)
	{
		/*if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13))
		{
			delay();
			GPIO_WriteToOutputPin(GPIOA, 5, 0);

		}
		else
		{
			delay();
			GPIO_WriteToOutputPin(GPIOA, 5, 1);
		}*/


		if(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13))		//Button already pulled-up
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, 5);
		}

	}
}

