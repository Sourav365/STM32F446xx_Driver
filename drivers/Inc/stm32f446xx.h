/*
 * stm32f446xx.h
 *
 *  Created on: Dec 17, 2022
 *      Author: Sourav Das
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#include <stdint.h>


/***************PROCESSOR SPECIFIC DETAILS***********/
/*
 * ARM CORTEX M-4 NVIC reg address
 */
#define NVIC_ISER0			((volatile uint32_t*) 0xE000E100U)
#define NVIC_ISER1			((volatile uint32_t*) 0xE000E104U)
#define NVIC_ISER2			((volatile uint32_t*) 0xE000E108U)
#define NVIC_ISER3			((volatile uint32_t*) 0xE000E10CU)

#define NVIC_ICER0			((volatile uint32_t*) 0xE000E180U)
#define NVIC_ICER1			((volatile uint32_t*) 0xE000E184U)
#define NVIC_ICER2			((volatile uint32_t*) 0xE000E188U)
#define NVIC_ICER3			((volatile uint32_t*) 0xE000E18CU)

#define NVIC_PRIORITY_ADDR	((volatile uint32_t*) 0xE000E400U)

#define NUMBER_PR_BITS_IMPLEMENTED		4





/***************MICROCONTROLLER SPECIFIC DETAILS***********/
/*
 * Base address of memories
 */
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x20001C00U
#define ROM_BASEADDR		0x1FFF0000U
#define SRAM_BASEADDR		SRAM1_BASEADDR

/////////////////////////////////////////////////////
/*
 * Base address of AHB,APB BUS peripherals
 */
#define PERIPH_BASEADDR		0x40000000U
#define APB1_BASEADDR		PERIPH_BASEADDR
#define APB2_BASEADDR		0x40010000U
#define AHB1_BASEADDR		0x40020000U
#define AHB2_BASEADDR		0x50000000U
//#define AHB3_BASEADDR		0x60000000U

///////////////////////////////////////////////////////
/*
 * Base address of peripherals hanging on APB1 BUS
 */
#define TIM2_BASEADDR		(APB1_BASEADDR+0x0000)
#define TIM3_BASEADDR		(APB1_BASEADDR+0x0400)
#define TIM4_BASEADDR		(APB1_BASEADDR+0x0800)
#define TIM5_BASEADDR		(APB1_BASEADDR+0x0C00)
#define TIM6_BASEADDR		(APB1_BASEADDR+0x1000)
#define TIM7_BASEADDR		(APB1_BASEADDR+0x1400)
#define TIM12_BASEADDR		(APB1_BASEADDR+0x1800)
#define TIM13_BASEADDR		(APB1_BASEADDR+0x1C00)
#define TIM14_BASEADDR		(APB1_BASEADDR+0x2000)

#define RTC_BKP_BASEADDR	(APB1_BASEADDR+0x2800)
#define WWDG_BASEADDR		(APB1_BASEADDR+0x2C00)
#define IWDG_BASEADDR		(APB1_BASEADDR+0x3000)

#define SPI2_BASEADDR		(APB1_BASEADDR+0x3800)
#define I2S2_BASEADDR		(APB1_BASEADDR+0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR+0x3C00)
#define I2S3_BASEADDR		(APB1_BASEADDR+0x3C00)
#define SPDIFRX_BASEADDR	(APB1_BASEADDR+0x4000)

#define USART2_BASEADDR		(APB1_BASEADDR+0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR+0x4800)
#define UART4_BASEADDR		(APB1_BASEADDR+0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR+0x5000)

#define I2C1_BASEADDR		(APB1_BASEADDR+0x5400)
#define I2C2_BASEADDR		(APB1_BASEADDR+0x5800)
#define I2C3_BASEADDR		(APB1_BASEADDR+0x5C00)
#define FMPI2C1_BASEADDR	(APB1_BASEADDR+0x6000)

#define CAN1_BASEADDR		(APB1_BASEADDR+0x6400)
#define CAN2_BASEADDR		(APB1_BASEADDR+0x6800)
#define HDMI_CEC_BASEADDR	(APB1_BASEADDR+0x6C00)
#define PWR_BASEADDR		(APB1_BASEADDR+0x7000)
#define DAC_BASEADDR		(APB1_BASEADDR+0x7400)


/*
 * Base address of peripherals hanging on APB2 BUS
 */
#define TIM1_BASEADDR		(APB2_BASEADDR+0x0000)
#define TIM8_BASEADDR		(APB2_BASEADDR+0x0400)

#define USART1_BASEADDR		(APB2_BASEADDR+0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR+0x1400)

#define ADC_BASEADDR		(APB2_BASEADDR+0x2000)////////////////??ADC1,2,3
#define SDIO_BASEADDR		(APB2_BASEADDR+0x2C00)
#define SPI1_BASEADDR		(APB2_BASEADDR+0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR+0x3400)
#define SYSCFG_BASEADDR		(APB2_BASEADDR+0x3800)
#define EXTI_BASEADDR		(APB2_BASEADDR+0x3C00)
#define TIM9_BASEADDR		(APB2_BASEADDR+0x4000)
#define TIM10_BASEADDR		(APB2_BASEADDR+0x4400)
#define TIM11_BASEADDR		(APB2_BASEADDR+0x4800)

#define SAI1_BASEADDR		(APB2_BASEADDR+0x5800)
#define SAI2_BASEADDR		(APB2_BASEADDR+0x5C00)


/*
 * Base address of peripherals hanging on AHB1 BUS
 */
#define GPIOA_BASEADDR		(AHB1_BASEADDR+0x0000)
#define GPIOB_BASEADDR		(AHB1_BASEADDR+0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR+0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR+0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR+0x1000)
#define GPIOF_BASEADDR		(AHB1_BASEADDR+0x1400)
#define GPIOG_BASEADDR		(AHB1_BASEADDR+0x1800)
#define GPIOH_BASEADDR		(AHB1_BASEADDR+0x1C00)

#define CRC_BASEADDR		(AHB1_BASEADDR+0x3000)
#define RCC_BASEADDR		(AHB1_BASEADDR+0x3800)
#define FLASH_INTR_BASEADDR	(AHB1_BASEADDR+0x3C00)
#define BKPSRAM_BASEADDR	(AHB1_BASEADDR+0x4000)

#define DMA1_BASEADDR		(AHB1_BASEADDR+0x6000)
#define DMA2_BASEADDR		(AHB1_BASEADDR+0x6400)

#define USB_OTG_HS_BASEADDR	0x40040000U

/*
 * Base address of peripherals hanging on AHB1 BUS
 */
#define USB_OTG_FS_BASEADDR	(AHB2_BASEADDR+0x0000)
#define DCMI_BASEADDR		(AHB2_BASEADDR+0x50000)




///////////////////////////////////////////////////////
/*
 * Peripheral register define for GPIOx
 */
typedef struct
{
	uint32_t volatile MODER;	//(GPIOx_BASEADDR+0X00)// GPIO port mode register
	uint32_t volatile OTYPER;	//(GPIOx_BASEADDR+0X04)// GPIO port output type register
	uint32_t volatile OSPEEDER;	//(GPIOx_BASEADDR+0X08)// GPIO port output speed register
	uint32_t volatile PUPDR;	//(GPIOx_BASEADDR+0X0C)// GPIO port pull-up/pull-down register
	uint32_t volatile IDR;		//(GPIOx_BASEADDR+0X10)// GPIO port input data register
	uint32_t volatile ODR;		//(GPIOx_BASEADDR+0X14)// GPIO port output data register
	uint32_t volatile BSRR;		//(GPIOx_BASEADDR+0X18)// GPIO port bit set/reset register
	uint32_t volatile LCKR;		//(GPIOx_BASEADDR+0X1C)// GPIO port configuration lock register
	uint32_t volatile AFR[2];	//(GPIOx_BASEADDR+0X20)// AFR[1]->GPIO alternate function high register, AFR[0]->GPIO alternate function low register
}GPIOx_Reg_t;


///////////////////////////////////////////////////////
/*
 * Registers define of Reset & Clock Control (RCC)
 */
typedef struct				//32-bit each
{
	uint32_t volatile CR;
	uint32_t volatile PLL_CFGR;
	uint32_t volatile CFGR;
	uint32_t volatile CIR;
	uint32_t volatile AHB1RSTR;
	uint32_t volatile AHB2RSTR;
	uint32_t volatile AHB3RSTR;
	uint32_t RESERVED1;
	uint32_t volatile APB1RSTR;
	uint32_t volatile APB2RSTR;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	uint32_t volatile AHB1ENR;
	uint32_t volatile AHB2ENR;
	uint32_t volatile AHB3ENR;
	uint32_t RESERVED4;
	uint32_t volatile APB1ENR;
	uint32_t volatile APB2ENR;
	uint32_t RESERVED5;
	uint32_t RESERVED6;
	uint32_t volatile AHB1LPENR;
	uint32_t volatile AHB2LPENR;
	uint32_t volatile AHB3LPENR;
	uint32_t RESERVED7;
	uint32_t volatile APB1LPENR;
	uint32_t volatile APB2LPENR;
	uint32_t RESERVED8;
	uint32_t RESERVED9;
	uint32_t volatile BDCR;
	uint32_t volatile CSR;
	uint32_t RESERVED10;
	uint32_t RESERVED11;
	uint32_t volatile SSCGR;
	uint32_t volatile PLLI2SCFGR;
	uint32_t volatile PLLSAICFGR;
	uint32_t volatile DCKCFGR;
	uint32_t volatile CKGATENR;
	uint32_t volatile DCKCFR2;
}RCC_Reg_t;


/*
 * Registers define of EXTI reg.
 */
typedef struct
{
	uint32_t volatile IMR;		//Interrupt Mask Reg.
	uint32_t volatile EMR;		//Event Mask Reg.
	uint32_t volatile RTSR;		//Rising Trigger Selection Reg.
	uint32_t volatile FTSR;		//Falling Trigger Selection Reg.
	uint32_t volatile SWIER;	//Software Interrupt Event Reg.
	uint32_t volatile PR;		//Pending Reg.
}EXTI_Reg_t;

/*
 * Registers define of SYSCFG reg.
 */
typedef struct
{
	uint32_t volatile MEMRMP;		//memory remap register
	uint32_t volatile PCM;			//peripheral mode configuration register
	uint32_t volatile EXTICR[4];	//external interrupt configuration register-1,2,3,4
	uint32_t volatile CMPCR;		//Compensation cell control register
	uint32_t volatile CFGR;			//configuration register
}SYSCFG_Reg_t;

/*
 * Registers define of SPI
 */
typedef struct
{
	uint32_t volatile CR1;			//Control reg. 1
	uint32_t volatile CR2;			//Control reg. 2
	uint32_t volatile SR;			//Status reg.
	uint32_t volatile DR;			//Data reg.
	uint32_t volatile CRCPR;		//CRC polynomial reg.
	uint32_t volatile RXCRCR;		//RX CRC reg.
	uint32_t volatile TXCRCR;		//TX CRC reg.
	uint32_t volatile I2SCFGR;		//I2S config reg.
	uint32_t volatile I2SPR;		//I2S prescaler reg
}SPI_Reg_t;

/*
 * Registers define of I2C
 */
typedef struct
{
	uint32_t volatile CR1;			//Control reg. 1
	uint32_t volatile CR2;			//Control reg. 2
	uint32_t volatile OAR1;			//Own Address reg. 1
	uint32_t volatile OAR2;			//Own Address reg. 2
	uint32_t volatile DR;			//Data reg.
	uint32_t volatile SR1;			//Status reg. 1
	uint32_t volatile SR2;			//Status Reg. 2
	uint32_t volatile CCR;			//Clock Ctrl reg.
	uint32_t volatile TRISE;		//TRISE Reg.
	uint32_t volatile FLTR;			//FLTR Reg.

}I2C_Reg_t;

/*
 * Registers define of USART
 */
typedef struct
{
	uint32_t volatile SR;			//Status reg.
	uint32_t volatile DR;			//Data reg.
	uint32_t volatile BRR;			//Baud rate reg.
	uint32_t volatile CR1;			//Control reg. 1
	uint32_t volatile CR2;			//Control reg. 2
	uint32_t volatile CR3;			//Control reg. 3
	uint32_t volatile GTPR;			//Guard time & prescaler reg.
}USART_Reg_t;

/*
 * Peripheral address typecasted to GPIOx_Reg_t data type
 */
#define GPIOA			((GPIOx_Reg_t*) GPIOA_BASEADDR)
#define GPIOB			((GPIOx_Reg_t*) GPIOB_BASEADDR)
#define GPIOC			((GPIOx_Reg_t*) GPIOC_BASEADDR)
#define GPIOD			((GPIOx_Reg_t*) GPIOD_BASEADDR)
#define GPIOE			((GPIOx_Reg_t*) GPIOE_BASEADDR)
#define GPIOF			((GPIOx_Reg_t*) GPIOF_BASEADDR)
#define GPIOG			((GPIOx_Reg_t*) GPIOG_BASEADDR)
#define GPIOH			((GPIOx_Reg_t*) GPIOH_BASEADDR)

#define RCC				((RCC_Reg_t*)	RCC_BASEADDR)

#define EXTI			((EXTI_Reg_t*)	EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_Reg_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_Reg_t*)	SPI1_BASEADDR)
#define SPI2			((SPI_Reg_t*)	SPI2_BASEADDR)
#define SPI3			((SPI_Reg_t*)	SPI3_BASEADDR)
#define SPI4			((SPI_Reg_t*)	SPI4_BASEADDR)

#define I2C1			((I2C_Reg_t*)	I2C1_BASEADDR)
#define I2C2			((I2C_Reg_t*)	I2C2_BASEADDR)
#define I2C3			((I2C_Reg_t*)	I2C3_BASEADDR)

#define USART1			((USART_Reg_t*) USART1_BASEADDR)
#define USART2			((USART_Reg_t*) USART2_BASEADDR)
#define USART3			((USART_Reg_t*) USART3_BASEADDR)
#define UART4			((USART_Reg_t*) UART4_BASEADDR)
#define UART5			((USART_Reg_t*) UART5_BASEADDR)
#define USART6			((USART_Reg_t*) USART6_BASEADDR)

/**************************ENABLE CLOCK FOR PERIPHERALS***************************/

/*
 * Macro for enable clk for GPIOx peripheral
 */
#define GPIOA_PCLK_EN()			((RCC->AHB1ENR) |= (1<<0))
#define GPIOB_PCLK_EN()			((RCC->AHB1ENR) |= (1<<1))
#define GPIOC_PCLK_EN()			((RCC->AHB1ENR) |= (1<<2))
#define GPIOD_PCLK_EN()			((RCC->AHB1ENR) |= (1<<3))
#define GPIOE_PCLK_EN()			((RCC->AHB1ENR) |= (1<<4))
#define GPIOF_PCLK_EN()			((RCC->AHB1ENR) |= (1<<5))
#define GPIOG_PCLK_EN()			((RCC->AHB1ENR) |= (1<<6))
#define GPIOH_PCLK_EN()			((RCC->AHB1ENR) |= (1<<7))

/*
 * Macro for enable clk for SPIx peripheral
 */
#define SPI1_PCLK_EN()			((RCC->APB2ENR) |= (1<<12))
#define SPI2_PCLK_EN()			((RCC->APB1ENR) |= (1<<14))
#define SPI3_PCLK_EN()			((RCC->APB1ENR) |= (1<<15))
#define SPI4_PCLK_EN()			((RCC->APB2ENR) |= (1<<13))

/*
 * Macro for enable clk for I2Cx peripheral
 */
#define I2C1_PCLK_EN()			((RCC->APB1ENR) |= (1<<21))
#define I2C2_PCLK_EN()			((RCC->APB1ENR) |= (1<<22))
#define I2C3_PCLK_EN()			((RCC->APB1ENR) |= (1<<23))

/*
 * Macro for enable clk for UARTx peripheral
 */
#define USART1_PCLK_EN()			((RCC->APB2ENR) |= (1<<4))
#define USART2_PCLK_EN()			((RCC->APB1ENR) |= (1<<17))
#define USART3_PCLK_EN()			((RCC->APB1ENR) |= (1<<18))
#define UART4_PCLK_EN()				((RCC->APB1ENR) |= (1<<19))
#define UART5_PCLK_EN()				((RCC->APB1ENR) |= (1<<20))
#define USART6_PCLK_EN()			((RCC->APB2ENR) |= (1<<5))

/*
 * Macro for enable clk for CANx peripheral
 */
#define CAN1_PCLK_EN()			((RCC->APB1ENR) |= (1<<25))
#define CAN2_PCLK_EN()			((RCC->APB1ENR) |= (1<<26))

/*
 * Macro for enable clk for DAC peripheral
 */
#define DAC_PCLK_EN()			((RCC->APB1ENR) |= (1<<29))

/*
 * Macro for enable clk for ADCx peripheral
 */
#define ADC1_PCLK_EN()			((RCC->APB2ENR) |= (1<<8))
#define ADC2_PCLK_EN()			((RCC->APB2ENR) |= (1<<9))
#define ADC3_PCLK_EN()			((RCC->APB2ENR) |= (1<<10))

/*
 * Macro for enable clk for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()			((RCC->APB2ENR) |= (1<<14))




/**************************DISABLE CLOCK FOR PERIPHERALS***************************/

/*
 * Macro for Disable clk for GPIOx peripheral
 */
#define GPIOA_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<0))
#define GPIOB_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<1))
#define GPIOC_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<2))
#define GPIOD_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<3))
#define GPIOE_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<4))
#define GPIOF_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<5))
#define GPIOG_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<6))
#define GPIOH_PCLK_DI()			((RCC->AHB1ENR) &= ~(1<<7))

/*
 * Macro for Disable clk for SPIx peripheral
 */
#define SPI1_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<12))
#define SPI2_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<14))
#define SPI3_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<15))
#define SPI4_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<13))

/*
 * Macro for Disable clk for I2Cx peripheral
 */
#define I2C1_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<21))
#define I2C2_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<22))
#define I2C3_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<23))

/*
 * Macro for Disable clk for UARTx peripheral
 */
#define USART1_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<4))
#define USART2_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<17))
#define USART3_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<18))
#define UART4_PCLK_DI()				((RCC->APB1ENR) &= ~(1<<19))
#define UART5_PCLK_DI()				((RCC->APB1ENR) &= ~(1<<20))
#define USART6_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<5))

/*
 * Macro for Disable clk for CANx peripheral
 */
#define CAN1_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<25))
#define CAN2_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<26))

/*
 * Macro for Disable clk for DAC peripheral
 */
#define DAC_PCLK_DI()			((RCC->APB1ENR) &= ~(1<<29))

/*
 * Macro for Disable clk for ADCx peripheral
 */
#define ADC1_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<8))
#define ADC2_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<9))
#define ADC3_PCLK_DI()			((RCC->APB2ENR) &= ~(1<<10))

/*
 * Macro for Disable clk for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()		((RCC->APB2ENR) &= ~(1<<14))

/*
 * Macro for Reset GPIOx peripheral
 */
#define GPIOA_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<0); (RCC->APB1RSTR) &= ~(1<<0);} while(0)
#define GPIOB_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<1); (RCC->APB1RSTR) &= ~(1<<1);} while(0)
#define GPIOC_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<2); (RCC->APB1RSTR) &= ~(1<<2);} while(0)
#define GPIOD_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<3); (RCC->APB1RSTR) &= ~(1<<3);} while(0)
#define GPIOE_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<4); (RCC->APB1RSTR) &= ~(1<<4);} while(0)
#define GPIOF_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<5); (RCC->APB1RSTR) &= ~(1<<5);} while(0)
#define GPIOG_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<6); (RCC->APB1RSTR) &= ~(1<<6);} while(0)
#define GPIOH_REG_RESET()		do {(RCC->AHB1RSTR) |= (1<<7); (RCC->APB1RSTR) &= ~(1<<7);} while(0)

#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 : 0 )


/*
 * Macro for Reset SPIx peripheral
 */
#define SPI1_REG_RESET()		do {(RCC->APB2RSTR) |= (1<<12); (RCC->APB2RSTR) &= ~(1<<12);} while(0)
#define SPI2_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<14); (RCC->APB1RSTR) &= ~(1<<14);} while(0)
#define SPI3_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<15); (RCC->APB1RSTR) &= ~(1<<15);} while(0)
#define SPI4_REG_RESET()		do {(RCC->APB2RSTR) |= (1<<13); (RCC->APB2RSTR) &= ~(1<<13);} while(0)

/*
 * Macro for Reset I2Cx peripheral
 */
#define I2C1_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<21); (RCC->APB2RSTR) &= ~(1<<21);} while(0)
#define I2C2_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<22); (RCC->APB1RSTR) &= ~(1<<22);} while(0)
#define I2C3_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<23); (RCC->APB1RSTR) &= ~(1<<23);} while(0)

/*
 * Macro for Reset USARTx peripheral
 */
#define USART1_REG_RESET()		do {(RCC->APB2RSTR) |= (1<< 4); (RCC->APB2RSTR) &= ~(1<< 4);} while(0)
#define USART2_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<17); (RCC->APB1RSTR) &= ~(1<<17);} while(0)
#define USART3_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<18); (RCC->APB1RSTR) &= ~(1<<18);} while(0)
#define UART4_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<19); (RCC->APB1RSTR) &= ~(1<<19);} while(0)
#define UART5_REG_RESET()		do {(RCC->APB1RSTR) |= (1<<20); (RCC->APB1RSTR) &= ~(1<<20);} while(0)
#define USART6_REG_RESET()		do {(RCC->APB2RSTR) |= (1<< 5); (RCC->APB2RSTR) &= ~(1<< 5);} while(0)

////////////////////////////////////////////////////////////////
/*
 * Some basic MACROS
 */
#define ENABLE					1
#define DISABLE					0
#define HIGH					ENABLE
#define LOW						DISABLE
#define SET						ENABLE
#define RESET					DISABLE
#define FLAG_RESET				RESET
#define FLAG_SET				SET
/*
 * Macros for IRQ(Interrupt Request) no.
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * Macros for possible priority levels
 */
#define NVIC_IRQ_PRIORITY0		0
#define NVIC_IRQ_PRIORITY1		1
#define NVIC_IRQ_PRIORITY2		2
#define NVIC_IRQ_PRIORITY3		3
#define NVIC_IRQ_PRIORITY4		4
#define NVIC_IRQ_PRIORITY5		5
#define NVIC_IRQ_PRIORITY6		6
#define NVIC_IRQ_PRIORITY7		7
#define NVIC_IRQ_PRIORITY8		8
#define NVIC_IRQ_PRIORITY9		9
#define NVIC_IRQ_PRIORITY10		10
#define NVIC_IRQ_PRIORITY11		11
#define NVIC_IRQ_PRIORITY12		12
#define NVIC_IRQ_PRIORITY13		13
#define NVIC_IRQ_PRIORITY14		14
#define NVIC_IRQ_PRIORITY15		15

/*
 * Macros for bit position of different reg. of SPI
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSB_FIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_OE			14
#define SPI_CR1_BIDI_MODE		15

#define SPI_CR2_RX_DMA_EN		0
#define SPI_CR2_TX_DMA_EN		1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRC_ERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/*
 * Macros for bit position of different reg. of I2C
 */
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERR_EN		8
#define I2C_CR2_ITEVT_EN		9
#define I2C_CR2_ITBUF_EN		10
#define I2C_CR2_DMA_EN			11
#define I2C_CR2_LAST			12


#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE 			7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PEC_ERR			12
#define I2C_SR1_TIME_OUT		14
#define I2C_SR1_SMB_ALERT		15


#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GEN_CALL		4
#define I2C_SR2_SMB_DEFAULT		5
#define I2C_SR2_SMB_HOST		6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

#define I2C_CCR_CCR 			0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS  			15

/*
 * Macros for bit position of different reg. of USART
 */
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NE				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC 			6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLE_IE		4
#define USART_CR1_RXNE_IE		5
#define USART_CR1_TC_IE			6
#define USART_CR1_TXE_IE		7
#define USART_CR1_PE_IE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M 			12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

#define USART_CR2_LBDL			5
#define USART_CR2_LBD_IE		6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLK_EN		11
#define USART_CR2_STOP			12
#define USART_CR2_LIN_EN		14

#define USART_CR3_EIE			0
#define USART_CR3_IR_EN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SC_EN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONE_BIT		11



//Other header files
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
