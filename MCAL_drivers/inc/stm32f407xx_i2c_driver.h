/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Jan 26, 2021
 *      Author: Nour
 */

#ifndef MCAL_DRIVERS_INC_STM32F407XX_I2C_DRIVER_H_
#define MCAL_DRIVERS_INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */


typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress; //mentioned by user
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_TypeDef *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t      *pTxBuffer;
	uint8_t      *pRxBuffer;
	uint32_t      TxLen;
	uint32_t      RxLen;
	uint8_t       TxRxState;
	uint8_t       DevAddr;  //slave/device address
	uint32_t      RxSize;
	uint8_t       Sr;		//repeated start
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 * IT'S arbitrary values
 */
#define I2C_SCL_SPEED_SM		1000000
#define I2C_SCL_SPEED_FM2K		2000000
#define I2C_SCL_SPEED_FM4K		4000000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_EN       		1
#define I2C_ACK_DI       		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * @Sr
 */
#define I2C_ENABLE_SR       		1
#define I2C_DISABLE_SR      		0

/*
 * @TxRxState
 */
#define I2C_READY		      		0
#define I2C_BUSY_IN_RX      		1
#define I2C_BUSY_IN_TX      		2

/*
 * I2C application events macros
 */
#define FLAG_MASK         ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */
/* --EV5 */
#define  I2C_EVENT_MASTER_MODE_SELECT                      ((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */
/* --EV6 */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED           ((uint32_t)0x00030002)  /* BUSY, MSL and ADDR flags */
/* --EV9 */
#define  I2C_EVENT_MASTER_MODE_ADDRESS10                   ((uint32_t)0x00030008)  /* BUSY, MSL and ADD10 flags */

#define I2C_EV_TX_CMPLT				0
#define I2C_EV_RX_CMPLT				1
#define I2C_EV_STOP					2

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7


/* Master RECEIVER mode -----------------------------*/
/* --EV7 */
#define  I2C_EVENT_MASTER_BYTE_RECEIVED                    ((uint32_t)0x00030040)  /* BUSY, MSL and RXNE flags */

/* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
/* --EV8_2 */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */


/****************************************************************************************
 * 								APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *****************************************************************************************/

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);


/***************************************************************************
 ********************* Data Send and Receive********************************
 ***************************************************************************/

void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data);
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);


/*
 * Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t Flag_Name);
//I2C enable
void I2C_Enable(I2C_TypeDef *pI2Cx, uint8_t En0rDi);

void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx);
void I2C_GenerateStopCondition(I2C_TypeDef *pI2Cx);

void I2C_ManageAcking(I2C_TypeDef *pI2Cx, uint8_t En0rDi);

//CHECK FLAG
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t Flag_Name);

ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);

void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr);
void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr);

/************************************************************************************
 *                           APIs interrupts
 *************************************************************************************/

/*
 * IRQ Cofiguration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

uint8_t I2C_SendDataIT(I2C_Handle_t *pI2Cx, uint8_t *pTxBuffer, uint32_t Len);
uint8_t I2C_ReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len);


void I2C_ClearOVRFlag(I2C_Handle_t *pI2CHandle);
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);
void I2C_CloseReception(I2C_Handle_t *pI2CHandle);



/*
 *Application call back
 */
void I2C_AppEventCallback(I2C_Handle_t *pHandle, uint8_t event);

#endif /* MCAL_DRIVERS_INC_STM32F407XX_I2C_DRIVER_H_ */
