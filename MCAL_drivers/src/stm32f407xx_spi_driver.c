/*
 * Peripheral Clock setup
 */

#include "stm32f407xx_spi_driver.h"

#include <string.h>



static void spi_txe_interrupt_handle();
static void spi_rxne_interrupt_handle();
static void spi_ovr_err_interrupt_handle();



/**************************************************************************************************
Usually, the SPI is connected to external devices through four pins:
� MISO: Master In / Slave Out data. This pin can be used to transmit data in slave mode
and receive data in master mode.

� MOSI: Master Out / Slave In data. This pin can be used to transmit data in master
mode and receive data in slave mode.

� SCK: Serial Clock output for SPI masters and input for SPI slaves.

� NSS: Slave select. This is an optional pin to select a slave device. This pin acts as a
�chip select� to let the SPI master communicate with slaves individually and to avoid
contention on the data lines. Slave NSS inputs can be driven by standard IO ports on
the master device. The NSS pin may also be used as an output if enabled (SSOE bit)
and driven low if the SPI is in master configuration. In this manner, all NSS pins from
devices connected to the Master NSS pin see a low level and become slaves when
they are configured in NSS hardware mode. When configured in master mode with
NSS configured as an input (MSTR=1 and SSOE=0) and if NSS is pulled low, the SPI
enters the master mode fault state: the MSTR bit is automatically cleared and the
device is configured in slave mode
 **************************************************************************************************/
/***************************************************************************************************
Slave select (NSS) pin management
Hardware or software slave select management can be set using the SSM bit in the
SPI_CR1 register.
� Software NSS management (SSM = 1)
The slave select information is driven internally by the value of the SSI bit in the
SPI_CR1 register. The external NSS pin remains free for other application uses.
� Hardware NSS management (SSM = 0)
Two configurations are possible depending on the NSS output configuration (SSOE bit
in register SPI_CR2).
� NSS output enabled (SSM = 0, SSOE = 1)
This configuration is used only when the device operates in master mode. The
NSS signal is driven low when the master starts the communication and is kept
low until the SPI is disabled.
� NSS output disabled (SSM = 0, SSOE = 0)
This configuration allows multimaster capability for devices operating in master
mode. For devices set as slave, the NSS pin acts as a classical NSS input: the
slave is selected when NSS is low and deselected when NSS high.
 ***************************************************************************************************/

/*
 * Peripheral Clock setup
 */
/***************************************************
 * @fn						-SPI_PeriClockControl
 *
 * @brief  					-enable or disable peripheral clock
 *
 * @param[in]				-base address of the spi peripheral
 * @param[in]               -ENABLE or DISABLE macros
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
		}
	}
}

/*
 * Init and De-init
 */
/***************************************************
 * @fn						-SPI_INIT
 *
 * @brief  					-Initializes the SPIx peripheral according to the specified parameters in the SPI_InitStruct.
 *
 * @param[in]				- pointer to a SPI_RegDef structure that contains
 *         				 the configuration information for the specified SPI peripheral.
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{




	//enable clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1
	pSPIHandle->pSPIx->CR1 = 0;
	//1. configure the device mode
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		pSPIHandle->pSPIx->CR1 |= (SPI_CR1_MSTR | SPI_CR1_SSI);
	}else
	{
		pSPIHandle->pSPIx->CR1 &= ~SPI_CR1_MSTR;
	}
	//2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bidi mode should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR1 &= ~(SPI_CR1_RXONLY);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Bidi mode should be set
		pSPIHandle->pSPIx->CR1 |= (SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//Bidi mode should be cleared
		pSPIHandle->pSPIx->CR1 &= ~(SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		pSPIHandle->pSPIx->CR1 |= (SPI_CR1_RXONLY);
	}

	//3. Configure the spi serial clock speed (baud rate)
	if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV2)
	{
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_2;
	}else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV4){
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_4;
	}else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV8){
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_8;
	}else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV16){
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_16;
	}else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV32){
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_32;
	}else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV64){
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_64;
	}else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV128){
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_128;
	}else{
		pSPIHandle->pSPIx->CR1 |= SPI_BAUDRATEPRESCALER_256;
	}


	//4. Configure the data frame format
	if(pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_8BITS)
	{
		pSPIHandle->pSPIx->CR1 &= ~(SPI_CR1_DFF);
	}else{
		pSPIHandle->pSPIx->CR1 |= (SPI_CR1_DFF);
	}

	//5. Configure the CPOL
	if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_LOW)
	{
		pSPIHandle->pSPIx->CR1 &= ~(SPI_CR1_CPOL);
	}else{
		pSPIHandle->pSPIx->CR1 |= (SPI_CR1_CPOL);
	}
	//6. configure the CPHA
	if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPHA_LOW)
	{
		pSPIHandle->pSPIx->CR1 &= ~(SPI_CR1_CPHA);
	}else{
		pSPIHandle->pSPIx->CR1 |= (SPI_CR1_CPHA);
	}

	//7. slave
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_Pos;
	pSPIHandle->pSPIx->CR2 = 0;

}



/***************************************************
 * @fn						-SPI_DEINIT
 *
 * @brief  					-Initializes the SPIx peripheral according to the specified parameters in the SPI_InitStruct.
 *
 * @param[in]				- pointer to a SPI_RegDef structure that contains
 *
 * @param[out]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void SPI_DeInit(SPI_TypeDef *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*
 * enable spi
 */

/***************************************************
 * @fn						-SPI_Enable
 *
 * @brief  					-enable or disable SPI.
 *
 * @param[in]				- pointer to a SPI_RegDef structure that contains
 *         				 		ENABLE or DISABLE macros
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void SPI_Enable(SPI_TypeDef *pSPIx, uint8_t En0rDi)
{
	if(En0rDi == ENABLE)
	{
		pSPIx->CR1 |= (SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(SPI_CR1_SPE);
	}
}

/*
 * SHOULD SSM software slave management bit is set
 * INTERNAL SLAVE SELECT
 */
/***************************************************
 * @fn						-SPI_SSIConfig
 *
 * @brief  					-enable or disable SPI.
 *
 * @param[in]				- pointer to a SPI_RegDef structure that contains
 *         				 		ENABLE or DISABLE macros
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t En0rDi)
{
	if(En0rDi == ENABLE)
	{
		pSPIx->CR1 |= (SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(SPI_CR1_SSI);
	}
}

/*
 * control ssoe bit (slave select output enable)
 * NSS output will be enabled when SSOE = 1
 */
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t En0rDi)
{
	if(En0rDi == ENABLE)
	{
		pSPIx->CR2 |= (SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(SPI_CR2_SSOE);
	}
}

/*
 * check flag
 */
uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx, uint32_t Flag_Name)
{
	if(pSPIx->SR & Flag_Name){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * Data Send
 */
/***************************************************
 * @fn						-SPI_SendData
 *
 * @brief  					-send data.
 *
 * @param[in]				- pointer to a SPI_TypeDef structure that contains
 *         				 		pointer to transmitted data
 * @param[in]               - length of data
 *
 *
 * @return 					- none
 *
 * @Note                    - without interrupt
 *
 */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint16_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_SR_TXE) == FLAG_RESET);

		//2. CHECK THE Data Frame Format bit in CR1
		if(pSPIx->CR1 & (SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load the data in the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len-=2; // we sent2 types of data
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *((uint8_t*)pTxBuffer);
			Len--;
			(uint8_t*)pTxBuffer++;
		}
	}
}

/*
 * Data Receive
 */
/***************************************************
 * @fn						-SPI_ReceiveData
 *
 * @brief  					-ReceiveData.
 *
 * @param[in]				- pointer to a SPI_RegDef structure that contains
 *         				 		pointer to transmitted data
 * @param[in]               - length of data
 *
 *
 * @return 					- none
 *
 * @Note                    - withOUT interrupt
 *
 */
void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint16_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_SR_RXNE) == FLAG_RESET);

		//2. CHECK THE Data Frame Format bit in CR1
		if(pSPIx->CR1 & (SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. read the data from the DR
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len-=2; // we sent2 types of data
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * IRQ Cofiguration and ISR handling
 */
/***************************************************
 * @fn						-SPI_IRQInterruptConfig
 *
 * @brief  					-enable or disable spi interrupt
 *
 * @param[in]				--
 *
 *
 * @return 					- none
 *
 * @Note                    - enable or disable interrupt
 *
 ***************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64));
		}

	}
}

/***************************************************
 * @fn						-SPI_IRQPriorityConfig
 *
 * @brief  					-configure priority of spi interrupt
 *
 * @param[in]				--
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 *******************************************************************/

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. FIRST find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //there is bits already implemented we can't write there

	*(NVIC_PR_BASE_ADDR + (4*iprx)) |= (IRQPriority << shift_amount );

}

/***************************************************
 * @fn						-SPI_SendDataIT
 *
 * @brief  					-send data.
 *
 * @param[in]				- pointer to a SPI_RegDef structure that contains
 *         				 		pointer to transmitted data
 * @param[in]               - length of data
 *
 *
 * @return 					- none
 *
 * @Note                    - with interrupt
 *
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark spi state as busy in transmission so that
		//   no other code can take over same SPI peripheral until trans. over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}

/***************************************************
 * @fn						-SPI_ReceiveDataIT
 *
 * @brief  					-receive data.
 *
 * @param[in]				- pointer to a SPI_Handle structure that contains
 *         				 		-pointer to transmitted data
 * @param[in]               - length of data
 *
 *
 * @return 					- none
 *
 * @Note                    - with interrupt
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark spi state as busy in transmission so that
		//   no other code can take over same SPI peripheral until recieve. over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_RXNEIE);

		//4. Data receiving will be handled by the ISR code
	}

	return state;
}

/***************************************************
 * @fn						-SPI_IRQHandling
 *
 * @brief  					-handle spi interrupt
 *
 * @param[in]				--pointer to a SPI_Handle structure that contains
 *
 *
 * @return 					- none
 *
 * @Note                    - we write this in ISR app function which we get its name from typedef enum IRQn in stm32f4xx.h
 *
 *******************************************/

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;
	//check for TXE
	temp1 = pHandle->pSPIx->SR & (SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (SPI_CR2_TXEIE);

	if( temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (SPI_CR2_RXNEIE);
	if( temp1 && temp2){
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}
	//check for ovr flag
	temp1 = pHandle->pSPIx->SR & (SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (SPI_CR2_ERRIE);
	if( temp1 && temp2){
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


//some helper function for handling iterrupts

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	if(pSPIHandle->pSPIx->CR1 & (SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load the data in the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen-=2; // we sent2 types of data
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( ! pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}



static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. CHECK THE Data Frame Format bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. read the data from the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2; // we sent2 types of data
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{
		//8 bit DFF
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if( ! pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);
		SPI_AppEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}




static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
		(void)temp;
	}
	SPI_ClearOVRFlag(pSPIHandle);
	SPI_AppEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	temp = pSPIHandle->pSPIx->DR;
	temp = pSPIHandle->pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_TXEIE); //close interrupts
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE); //close interrupts
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



__weak void SPI_AppEventCallback(SPI_Handle_t *pHandle, uint8_t event)
{

}

/******************************************************************************
 * FULL DUPLEX MODE
 1. Enable the SPI by setting the SPE bit to 1.
2. Write the first data item to be transmitted into the SPI_DR register (this clears the TXE
flag).
3. Wait until TXE=1 and write the second data item to be transmitted. Then wait until
RXNE=1 and read the SPI_DR to get the first received data item (this clears the RXNE
bit). Repeat this operation for each data item to be transmitted/received until the n�1
received data.
4. Wait until RXNE=1 and read the last received data.
5. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI.
This procedure can also be implemented using dedicated interrupt subroutines launched at
each rising edges of the RXNE or TXE flag.
 ***************************************************************************************/

/***********************************************************************************************
 *                               Disabling the SPI
 *                               In master or slave full-duplex mode (BIDIMODE=0, RXONLY=0)
1. Wait until RXNE=1 to receive the last data
2. Wait until TXE=1
3. Then wait until BSY=0
4. Disable the SPI (SPE=0) and, eventually, enter the Halt mode (or disable the peripheral
clock)                                                              *
 ***********************************************************************************************/
