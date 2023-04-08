/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 18-Dec-2022
 *      Author: Sourav Das
 */

#include "../Inc/stm32f446xx_gpio_driver.h"

/*****************************************************************
 * Peripheral Clock setup
 ****************************************************************/

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
void GPIO_PCLK(GPIOx_Reg_t *pGPIOx, uint8_t EnDi)
{
	if(EnDi){	//EnDi == ENABLE (1)
		if	   (pGPIOx == GPIOA)	GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)	GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)	GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)	GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)	GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)	GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)	GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)	GPIOH_PCLK_EN();
	}
	else{
		if	   (pGPIOx == GPIOA)	GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)	GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)	GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)	GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)	GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)	GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)	GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)	GPIOH_PCLK_DI();
	}
}






/*****************************************************************
 * Init & DeInit
 ****************************************************************/

/*****************************************************************
 * @function				-> GPIO_Init
 *
 * @brief					-> Initialize a given GPIO port and its pin
 *
 * @param1					->
 * @param2					->
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//0. Enable peripheral clk
	GPIO_PCLK(pGPIOHandle->pGPIOx, ENABLE);

	//1. Config pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  <=  GPIO_MODE_ANLOG)
	{
		//Non-interrupt/normal modes
		pGPIOHandle->pGPIOx->MODER &= ~(0x3  <<  (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));    //clearing bits/////////////////////////////no 2 in video/////////////////////////////////////////
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  <<  (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); //Shifted by double times as each mode 2-bits
	}
	else
	{
		//Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//i) Configure FTSR
			EXTI->FTSR	|= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear RTSR
			EXTI->RTSR	&= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//i) Configure RTSR
			EXTI->RTSR	|= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear FTSR
			EXTI->FTSR	&= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//i) Configure RTSR & FTSR
			EXTI->RTSR	|= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR	|= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//ii)  Config GPIO port selection in SYSCFG_EXTICR[0-3]
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portCode << 4*temp2;
		//iii) Enable EXTI interrupt delivery using IMR (Int. Mask Reg)
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//2. Config pin output type
	pGPIOHandle->pGPIOx->OTYPER  &=  ~(0x3  <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //clear
	pGPIOHandle->pGPIOx->OTYPER  |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType  <<  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));    //set

	//3. Config pin speed
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3  <<  (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->OSPEEDER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed  <<  (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));

	//4. Config pin pull-up, pull-down
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3  <<  (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl  <<  (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));

	//5. Config pin alternate functions
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  ==  GPIO_MODE_ALTERNET)
	{
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8;
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8;
		pGPIOHandle->pGPIOx->AFR[temp1]  &=  ~(0xF  <<  (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1]  |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode  <<  (4 * temp2));

	}

}

/*****************************************************************
 * @function				-> GPIO_DeInit
 *
 * @brief					-> Disable given GPIO peripheral
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					->
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_DeInit(GPIOx_Reg_t *pGPIOx)
{
	if     (pGPIOx == GPIOA)		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)		GPIOH_REG_RESET();
}







/*****************************************************************
 * Read-Write of data
 ****************************************************************/

/*****************************************************************
 * @function				-> GPIO_ReadFromInputPin
 *
 * @brief					-> Read data from given GPIO port pin
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> Desired pin number of the port
 *
 * @return					-> data read from pin (0/1)
 *
 * @note					-> none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)(((pGPIOx->IDR) >> PinNumber) & 0x1);
}

/*****************************************************************
 * @function				-> GPIO_ReadFromInputPort
 *
 * @brief					-> Read data from given GPIO port
 *
 * @param1					-> Base address of GPIO peripheral
 *
 * @return					-> data read from port (16-bit, 16 pins)
 *
 * @note					-> none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIOx_Reg_t *pGPIOx){
	return (uint16_t)(pGPIOx->IDR);
}

/*****************************************************************
 * @function				-> GPIO_WriteToOutputPin
 *
 * @brief					-> Write data to a given GPIO port pin
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> Desired output pin number of the port
 * @param3					-> Output value
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_WriteToOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value)	//value=1 -> set bit of ODR
		(pGPIOx->ODR) |= (1<<PinNumber);
	else		//value=0 -> reset bit of ODR
		(pGPIOx->ODR) &= ~(1<<PinNumber);
}

/*****************************************************************
 * @function				-> GPIO_WriteToOutputPort
 *
 * @brief					-> Write data to a given GPIO port
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> Output value (16-bit, 16 pins)
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_WriteToOutputPort(GPIOx_Reg_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/*****************************************************************
 * @function				-> GPIO_ToggleOutputPin
 *
 * @brief					-> Toggle output of a given GPIO port pin
 *
 * @param1					-> Base address of GPIO peripheral
 * @param2					-> Pin number of an output port
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_ToggleOutputPin(GPIOx_Reg_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*****************************************************************
 * IRQ config & ISR handling
 ****************************************************************/

/*****************************************************************
 * @function				-> GPIO_IRQ_Interrupt_Config
 *
 * @brief					-> Enable/Disable interrupt by NVIC ISER or ICER
 *
 * @param1					-> IRQ number of interrupt
 * @param2					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi)
{
	if(EnDi == ENABLE)
	{
		if(IRQNumber <=31)								//Program ISER0 reg.
			*NVIC_ISER0 |= (1 << IRQNumber);
		else if (IRQNumber >=32 && IRQNumber <=63)		//Program ISER1 reg.
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		else if (IRQNumber >=64 && IRQNumber <=95)		//Program ISER2 reg.
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
	}
	else
	{
		if(IRQNumber <=31)								//Program ICER0 reg.
			*NVIC_ICER0 |= (1 << IRQNumber);
		else if (IRQNumber >=32 && IRQNumber <=63)		//Program ICER1 reg.
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		else if (IRQNumber >=64 && IRQNumber <=95)		//Program ICER2 reg.
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
	}
}

/*****************************************************************
 * @function				-> GPIO_IRQ_Priority_Config
 *
 * @brief					-> Set priority of interrupt
 *
 * @param1					-> IRQ number of interrupt
 * @param2					-> IRQ priority number
 * @param3					-> ENABLE/DISABLE macros
 *
 * @return					-> none
 *
 * @note					-> none
 *
 */
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi)
{
	uint8_t iprx = IRQNumber / 4;		//Interrupt Priority reg.
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shiftAmount  = (8 * iprx_section) + (8 - NUMBER_PR_BITS_IMPLEMENTED);
	*((NVIC_PRIORITY_ADDR) + iprx) |= (IRQPriority << shiftAmount);
}

/*****************************************************************
 * @function				-> GPIO_IRQHandling
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
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear EXTI pending reg. corresponding to pin no.
	if(EXTI->PR & (1<<PinNumber)) //if pending
	{
		EXTI->PR |= (1<<PinNumber); //clear pending by setting pin
	}

}


