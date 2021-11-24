/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: Jan 30, 2021
 *      Author: Nour
 */

#ifndef MCAL_DRIVERS_INC_STM32F407XX_USART_DRIVER_H_
#define MCAL_DRIVERS_INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_TypeDef 	*pUSARTx;
	USART_Config_t   USART_Config;
	uint8_t       	*pTxBuffer; //for interrupt to store the app.Tx buffer address
	uint8_t       	*pRxBuffer; //for interrupt to store the app.rx buffer address
	uint32_t       	RxLen;
	uint32_t       	TxLen;
	uint8_t       	TxState; //for interrupt
	uint8_t       	RxState;
}USART_Handle_t;



/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*!< USART CR1 register clear Mask ((~(uint16_t)0xE9F3)) */
#define USART_CR1_CLEAR_MASK            ((uint16_t)(USART_CR1_M | USART_CR1_PCE | \
                                              USART_CR1_PS | USART_CR1_TE | \
                                              USART_CR1_RE))

/*!< USART CR2 register clock bits clear Mask ((~(uint16_t)0xF0FF)) */
#define USART_CR2_CLOCK_CLEAR_MASK      ((uint16_t)(USART_CR2_CLKEN | USART_CR2_CPOL | \
                                              USART_CR2_CPHA | USART_CR2_LBCL))

/*!< USART CR3 register clear Mask ((~(uint16_t)0xFCFF)) */
#define USART_CR3_CLEAR_MASK            ((uint16_t)(USART_CR3_RTSE | USART_CR3_CTSE))

/*!< USART Interrupts mask */
#define USART_IT_MASK                   ((uint16_t)0x001F)

/*
 * possible USART application states
 */
#define USART_READY							0
#define USART_BUSY_IN_RX					1
#define USART_BUSY_IN_TX					2

/*
 * possible USART application EVENTS
 */
#define USART_EVENT_TX_CMPLT				0
#define USART_EVENT_RX_CMPLT				1
#define USART_EVENT_CTS						2
#define USART_EVENT_IDLE					3
/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);
void USART_Enable(USART_TypeDef *pUSARTx, uint8_t En0rDi);

/*
 * Data Send and Receive
 */
void USART_SendChar(USART_Handle_t *pUSARTHandle, uint8_t data);
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer);
//uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t Len);
/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_TypeDef *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pHandle, uint8_t event);

#endif /* MCAL_DRIVERS_INC_STM32F407XX_USART_DRIVER_H_ */
