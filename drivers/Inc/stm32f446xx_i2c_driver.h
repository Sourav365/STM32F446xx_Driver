/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 03-Jan-2023
 *      Author: dsour
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCL_Speed;			//!<Possible values @I2C_SCL_Speed
	uint8_t  I2C_Device_Address;	//!<7-bit value given by user
	uint8_t  I2C_ACK_Control;		//!<Possible values @I2C_ACK_Control
	uint16_t I2C_FM_DutyCycle;		//!<Possible values @I2C_FM_DutyCycle
}I2C_Config_t;

/*
 * Handle structure of I2Cx peripheral
 */
typedef struct
{
	I2C_Reg_t		*pI2Cx;			//Base address
	I2C_Config_t	I2C_Config; 	//For configuration
}I2C_Handle_t;

/*
 * @I2C_SCL_Speed
 */
#define I2C_SCL_SPEED_NORMAL		100000
#define I2C_SCL_SPEED_FM2K			200000
#define I2C_SCL_SPEED_FM4K			400000

/*
 * @I2C_ACK_Control
 */
#define I2C_ACK_EN					1
#define I2C_ACK_DI					0

/*
 * @I2C_FM_DutyCycle
 */
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

/*
 * I2C status flag
 */

#define I2C_FLAG_RXNE					(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE					(1 << I2C_SR1_TXE)
#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOP					(1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR					(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
#define I2C_FLAG_PEC_ERR				(1 << I2C_SR1_PEC_ERR)
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIME_OUT)
#define I2C_FLAG_SMBALERT				(1 << I2C_SR1_SMB_ALERT)



/****************************************
 * 			Supported APIs
 ***************************************/

/*
 * Peripheral Clock setup
 */
void 	 I2C_PCLK(I2C_Reg_t *pI2Cx, uint8_t EnDi);

/*
 * Init & DeInit
 */
void 	 I2C_Init(I2C_Handle_t *pI2CHandle);
void 	 I2C_DeInit(I2C_Reg_t *pI2Cx);

/*
 * Data send and receive
 */
void 	 I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlavaAddr);
void 	 I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t SlaveAddr);
/*
 * IRQ config & ISR handling
 */
void	 I2C_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EnDi);
void 	 I2C_IRQ_Priority_Config(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnDi);


/*
 * Other peripheral control APIs
 */
uint8_t  I2C_GetFlagStatus(I2C_Reg_t *pI2Cx, uint32_t FlagName);
void 	 I2C_Peripheral_Control(I2C_Reg_t *pI2C, uint8_t EnDi);
void 	 I2C_Manage_Acking(I2C_Reg_t *pI2Cx, uint8_t EnDi);

/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
