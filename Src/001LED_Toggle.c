#include "../drivers/Inc/stm32f446xx.h"

void delay()
{
	for(uint32_t i=0; i<=300000;i++);
}


/* 0. Specifications needed
 * 		-> Address	: GPIOA
 * 		-> pin no	: 5
 * 		-> pin mode	: OUTPUT
 * 		-> out type	: PUSHPULL
 * 		-> out speed: MEDIUM
 * 		-> pupd reg : NO (as pushpull enabled)
 *
 * 1. enable clk for peripheral
 * 2. initiate peripheral according to specifications
 * 3. call APIs to read/write....
 */
int main()
{
//	 0. Specifications needed
	GPIO_Handle_t GpioLed;

//	  		-> Address	: GPIOA
	GpioLed.pGPIOx = GPIOA;

//	  		-> pin no	: 5
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;

//	  		-> pin mode	: OUTPUT
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;

//	  		-> out type	: PUSHPULL
	GpioLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OP_TYPE_PUSHPULL;

//	  		-> out speed: MEDIUM
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;

//	  		-> pupd reg : NO (as pushpull enabled)
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPDCTRL_NO;


//
//	  1. enable clk for peripheral
	GPIO_PCLK(GPIOA, ENABLE);

//	  2. initiate peripheral according to specifications
	GPIO_Init(&GpioLed);

//	  3. call APIs to read/write....
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}
}
