/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 26, 2020
 *      Author: Nour
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_



#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"

/*
 * This is a Configuration structure for a SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode; 					/*@SPI_DeviceMode*/
	uint8_t SPI_BusConfig;						/*@BusConfig full duplex or half*/
	uint8_t SPI_SclkSpeed;						/*@SPI_SclkSpeed*/
	uint8_t SPI_DFF;							/*@SPI_DFF data format*/
	uint8_t SPI_CPOL;							/*@CPOL*/
	uint8_t SPI_CPHA;							/*@CPHA*/
	uint8_t SPI_SSM;							/*@SSM hardware or software*/

}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_TypeDef  *pSPIx;
	SPI_Config_t  SPIConfig;
	uint8_t       *pTxBuffer; //for interrupt to store the app.Tx buffer address
	uint8_t       *pRxBuffer; //for interrupt to store the app.rx buffer address
	uint32_t       RxLen;
	uint32_t       TxLen;
	uint8_t       TxState; //for interrupt
	uint8_t       RxState;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER					1
#define SPI_DEVICE_MODE_SLAVE					0


/*
 * @BusConfig
 */
#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3


/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS    0
#define SPI_DFF_16BITS   1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH     1
#define SPI_CPOL_LOW      0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH     1
#define SPI_CPHA_LOW      0

/*
 * @SSM
 */
#define SPI_SSM_EN       1
#define SPI_SSM_DI       0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG      (SPI_SR_TXE)
#define SPI_RXNE_FLAG     (SPI_SR_RXNE)
#define SPI_BUSY_FLAG     (SPI_SR_BSY)

/*
 * possible spi application states
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/*
 * possible SPT APP Event
 */
#define SPI_EVENT_TX_CMPLT              1
#define SPI_EVENT_RX_CMPLT              2
#define SPI_EVENT_OVR_ERR               3

/****************************************************************************************
 * 								APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *****************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);

//SPI enable
void SPI_Enable(SPI_TypeDef *pSPIx, uint8_t En0rDi);

/*
 * SHOULD SSM software slave management bit is set
 * INTERNAL SLAVE SELECT
 */
void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t En0rDi);

/*
 * control ssoe bit (slave select output enable)
 * NSS output will be enabled when SSOE = 1
 */
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t En0rDi);

//CHECK FLAG
uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t Flag_Name);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint16_t Len);
void SPI_ReceiveData(SPI_TypeDef *pSPIHandle, uint8_t *pRxBuffer, uint16_t Len);

/*
 * Data Send and Receive for interrupt
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * IRQ Cofiguration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 *Application call back
 */
void SPI_AppEventCallback(SPI_Handle_t *pHandle, uint8_t event);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
