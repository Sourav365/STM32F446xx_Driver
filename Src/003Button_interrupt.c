/*
 * 003Button_interrupt.c
 *
 *  Created on: Dec 21, 2022
 *      Author: dsour
 */
#include <string.h>
#include "../drivers/Inc/stm32f446xx.h"

int main()
{
	GPIO_Handle_t GpioLed, GpioButton;

	//Initialize all as zero
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioButton,0,sizeof(GpioButton));

	//Config for GpioLed
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;

	//Config for GpioButton
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	//GpioButton.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPDCTRL_PULLUP; //already pulledup

	//Enable clk
	GPIO_PCLK(GPIOA, ENABLE);
	GPIO_PCLK(GPIOC, ENABLE);

	//Initiate peripheral
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//IRQ config
	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY3, ENABLE); //OPTIONAL
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI15_10, ENABLE);
}

void EXTI15_10_IRQHandler()
{
	for(uint32_t i=0;i<=500000;i++);
	GPIO_IRQHandling(GPIO_PIN_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
}


