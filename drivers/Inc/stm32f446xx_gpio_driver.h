/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 18-Dec-2022
 *      Author: Sourav Das
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure of GPIO Pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			//!<Possible values @GPIO_PIN_NUMBER>
	uint8_t GPIO_PinMode;			//!<Possible values @GPIO_MODE>
	uint8_t GPIO_PinSpeed;			//!<Possible values @GPIO_output_speed>
	uint8_t GPIO_PinPuPdCtrl;		//!<Possible values @GPIO_Pull-up_Pull-down>
	uint8_t GPIO_PinOutType;		//!<Possible values @GPIO_Output_type>
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handle structure of GPIO Pin
 */
typedef struct
{
	GPIOx_Reg_t *pGPIOx;				//GPIO registers
	GPIO_PinConfig_t GPIO_PinConfig;	//Possible values of GPIO reg.
}GPIO_Handle_t;



/*
 * Other Macros
 */

//@GPIO_PIN_NUMBER
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

//@GPIO_MODE reg
#define GPIO_MODE_INPUT				0	//Input mode  (reset)
#define GPIO_MODE_OUTPUT			1	//Output mode
#define GPIO_MODE_ALTERNET			2	//Alternate function mode
#define GPIO_MODE_ANLOG				3	//Analog mode

#define GPIO_MODE_IT_FT				4	//Interrupt-Falling edge
#define GPIO_MODE_IT_RT				5	//Interrupt-Rising edge
#define GPIO_MODE_IT_RFT			6	//Interrupt-Rising-Falling edge

//@GPIO_Output_type reg
#define GPIO_OP_TYPE_PUSHPULL		0	//Push pull (reset)
#define GPIO_OP_TYPE_OPENDRAIN		1	//Push pull

//@GPIO_output_speed reg
#define GPIO_OP_SPEED_LOW			0
#define GPIO_OP_SPEED_MEDIUM		1
#define GPIO_OP_SPEED_FAST			2
#define GPIO_OP_SPEED_HIGH			3

//@GPIO_Pull-up_Pull-down reg
#define GPIO_PUPDCTRL_NO				0	//No pullup, pulldown (reset)
#define GPIO_PUPDCTRL_PULLUP			1	//Pullup activate
#define GPIO_PUPDCTRL_PULLDOWN			2	//Pulldown activate


/****************************************
 * 			Supported APIs
 ***************************************/
//Peripheral Clock setup
void 	 GPIO_PCLK(GPIOx_Reg_t *pGPIOx, uint8_t EnDi);

//Init & DeInit
void 	 GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void 	 GPIO_DeInit(GPIOx_Reg_t *pGPIOx);

//Read-Write of data
uint8_t  GPIO_ReadFromInputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_Reg_t *pGPIOx);
void 	 GPIO_WriteToOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void 	 GPIO_WriteToOutputPort(GPIOx_Reg_t *pGPIOx, uint16_t value);
void     GPIO_ToggleOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNumber);

//IRQ config & ISR handling
void	 GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void 	 GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);
void 	 GPIO_IRQHandling(uint8_t PinNumber);






#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
