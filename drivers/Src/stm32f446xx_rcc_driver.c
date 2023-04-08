/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Jan 13, 2023
 *      Author: dsour
 */

#include "../Inc/stm32f446xx_rcc_driver.h"

uint32_t RCC_GetPLLOutClk() /////////////////////////////////not implemented!
{
	return 0;
}

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_Prescaler[4] = {2,4,8,16};
uint32_t RCC_GetPclk1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clk_src = (RCC->CFGR >> 2) & 0x3;
	uint8_t ahbPrescaler, apb1Prescaler;

	if (clk_src == 0) 		SystemClk=16000000; //HSI
	else if(clk_src == 1)	SystemClk=8000000;  //HSE
	else if(clk_src == 2)	SystemClk=RCC_GetPLLOutClk();  //PLL

	//for AHB
	uint8_t temp = ((RCC->CFGR >>4) & 0xF);
	if(temp < 8) ahbPrescaler = 1;
	else ahbPrescaler = AHB_Prescaler[temp-8];

	//For APB1
	temp = (RCC->CFGR >> 10) & 0x7;
	if(temp < 4) apb1Prescaler = 1;
	else apb1Prescaler = APB1_Prescaler[temp-4];

	pclk1 = (SystemClk/ahbPrescaler)/apb1Prescaler;
	return pclk1;
}



uint32_t RCC_GetPclk2Value(void)
{
	uint32_t pclk2,SystemClk;
	uint8_t clk_src = (RCC->CFGR >> 2) & 0x3;
	uint8_t ahbPrescaler, apb1Prescaler;

	if (clk_src == 0) 		SystemClk=16000000; //HSI
	else if(clk_src == 1)	SystemClk=8000000;  //HSE
	else if(clk_src == 2)	SystemClk=RCC_GetPLLOutClk();  //PLL

	//for AHB
	uint8_t temp = ((RCC->CFGR >>4) & 0xF);
	if(temp < 8) ahbPrescaler = 1;
	else ahbPrescaler = AHB_Prescaler[temp-8];

	//For APB1
	temp = (RCC->CFGR >> 10) & 0x7;
	if(temp < 4) apb1Prescaler = 1;
	else apb1Prescaler = APB1_Prescaler[temp-4];

	pclk1 = (SystemClk/ahbPrescaler)/apb1Prescaler;
	return pclk1;
}
