/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: 03-Jan-2023
 *      Author: dsour
 */

#include "../Inc/stm32f446xx_i2c_driver.h"
#include "../Inc/stm32f446xx_rcc_driver.h"

/*
 * Peripheral Clock setup
 */
void I2C_PCLK(I2C_Reg_t *pI2Cx, uint8_t EnDi)
{
	if(EnDi){	//EnDi == ENABLE (1)
			if	   (pI2Cx == I2C1)		I2C1_PCLK_EN();
			else if(pI2Cx == I2C2)		I2C2_PCLK_EN();
			else if(pI2Cx == I2C3)		I2C3_PCLK_EN();
		}
	else{
			if	   (pI2Cx == I2C1)		I2C1_PCLK_DI();
			else if(pI2Cx == I2C2)		I2C2_PCLK_DI();
			else if(pI2Cx == I2C3)		I2C3_PCLK_DI();
		}
}

/*
 * Init & DeInit
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;

	//0. Initiate clk
	I2C_PCLK(pI2CHandle->pI2Cx, ENABLE);

	//1. Configure mode (standard, fast)
	//CCR calculation
	uint16_t ccr_val=0; tempreg=0;
	if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_NORMAL)
	{
		//Standard mode
		ccr_val = (RCC_GetPclk1Value()/(2*pI2CHandle->I2C_Config.I2C_SCL_Speed));
		tempreg |= (ccr_val & 0xFFF);
	}
	else
	{
		//Fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FM_DutyCycle << I2C_CCR_DUTY);

		if (pI2CHandle->I2C_Config.I2C_FM_DutyCycle == I2C_FM_DUTY_2)
			ccr_val = RCC_GetPclk1Value()/(3*pI2CHandle->I2C_Config.I2C_SCL_Speed);
		else
			ccr_val = RCC_GetPclk1Value()/(25*pI2CHandle->I2C_Config.I2C_SCL_Speed);

		tempreg |= (ccr_val & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//2. Configure speed
	tempreg=0;
	tempreg |= (RCC_GetPLLOutClk()/1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//3. Configure device address
	tempreg=0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_Device_Address << 1);
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//4. Enable acking
	tempreg=0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACK_Control << I2C_CR1_ACK);

	//5. Configure rise time for I2C
	if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_NORMAL)
		{
			//Standard mode
			tempreg = (RCC_GetPclk1Value()/1000000U)+1;
		}
		else
		{
			//Fast mode
			tempreg = ((RCC_GetPclk1Value() *300)/1000000000U)+1;
		}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}


void I2C_DeInit(I2C_Reg_t *pI2Cx)
{
	if     (pI2Cx == I2C1)		I2C1_REG_RESET();
	else if(pI2Cx == I2C2)		I2C2_REG_RESET();
	else if(pI2Cx == I2C3)		I2C3_REG_RESET();
}

/*
 * Data send and receive
 */
static void I2C_GenerateStartCondition(I2C_Reg_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ExecuteAddressPhaseWrite(I2C_Reg_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);	//Reset 1st bit for Write
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_Reg_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1); 	//Set 1st bit for Read
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearADDRFlag(I2C_Reg_t *pI2Cx)	////????????????????????????????????????????????????
{
	uint32_t tempRead = pI2Cx->SR1;
	tempRead = pI2Cx->SR2;
	(void)tempRead;
}
static void I2C_GenerateStopCondition(I2C_Reg_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	//1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking SB flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send address of slave with R/Wb bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm address phase is completed by checking ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear ADDR Flag-> Until ADDR cleared, SCL will be stretched
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send data until length becomes 0
	while(len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//Wait until TXE is set
		pI2CHandle->pI2Cx->DR = *(pTxBuffer);
		pTxBuffer++;
		len--;
	}

	//7. Wait for TXE=1 and BTF=1 before STOP condition
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//Wait until TXE is set
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));//Wait until BTE is set

	//8. Generate stop condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	//1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking SB flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send address of slave with R/Wb bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4.  Confirm address phase is completed by checking ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//Read 1 byte only from slave
	if(len == 1)
	{
		//Disable acking
		I2C_Manage_Acking(pI2CHandle->pI2Cx,I2C_ACK_DI);

		//generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	//Read morethan 1 Byte from slave
	if(len > 1)
	{
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//Read data until length becomes 0
		for(uint32_t i=len; i>0; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			//If last 2 byte remains
			if(i==2)
			{
				//clear Ack bit
				I2C_Manage_Acking(pI2CHandle->pI2Cx,I2C_ACK_DI);

				//Generate stop bit
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read data from data reg to buffer adddr
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//Increament buffer addr
			pRxBuffer++;
		}
	}

	//Re-enable acking
	if(pI2CHandle->I2C_Config.I2C_ACK_Control == I2C_ACK_EN)
		I2C_Manage_Acking(pI2CHandle->pI2Cx,I2C_ACK_EN);

}
/*
 * IRQ config & ISR handling
 */
void	 I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void 	 I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);


/*
 * Other peripheral control APIs
 */
uint8_t  I2C_GetFlagStatus(I2C_Reg_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName) return FLAG_SET;
	else					  return FLAG_RESET;
}

void 	 I2C_Peripheral_Control(I2C_Reg_t *pI2Cx, uint8_t EnDi)
{
	if(EnDi == ENABLE)	pI2Cx->CR1 |=  (1 << I2C_CR1_PE);
	else				pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
}

void I2C_Manage_Acking(I2C_Reg_t *pI2Cx, uint8_t EnDi)
{
	if(EnDi == I2C_ACK_EN)
	{
		//Enable Ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else{
		//Disable Ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
