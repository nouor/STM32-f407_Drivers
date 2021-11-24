/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jan 26, 2021
 *      Author: Nour
 */

#include "stm32f407xx_i2c_driver.h"


/***************************************************
 * @fn						-I2C_PeriClockControl
 *
 * @brief  					-enable or disable peripheral clock
 *
 * @param[in]				-base address of the I2C peripheral
 * @param[in]               -ENABLE or DISABLE macros
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
	}
}




/*
 * Init and De-init
 */
/***************************************************
 * @fn						-I2C_INIT
 *
 * @brief  					-Initializes the I2Cx peripheral according to the specified parameters in the I2C_InitStruct.
 *
 * @param[in]				- pointer to a I2C_Handle_t structure that contains
 *         				 the configuration information for the specified I2C peripheral.
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//enable i2c
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK_Pos;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configuration the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address (only 7 bit)
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD1_Pos;
	tempreg |= I2C_OAR1_ADD4;
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//T*high*(scl)=CCR*Tpclk1, T*low*(scl)=CCR*Tpclk1
		//standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else
	{
	    //fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/***************************************************
 * @fn						-I2C_DEINIT
 *
 * @brief  					-Initializes the I2Cx peripheral according to the specified parameters in the I2C_InitStruct.
 *
 * @param[in]				- pointer to a I2C_RegDef structure that contains
 *
 * @param[out]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

}

/***************************************************
 * @fn						-I2C_Enable
 *
 * @brief  					-enable or disable I2C.
 *
 * @param[in]				- pointer to a I2C_RegDef structure that contains
 *         				 		ENABLE or DISABLE macros
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void I2C_Enable(I2C_TypeDef *pI2Cx, uint8_t En0rDi)
{
	if(En0rDi == ENABLE)
	{
		pI2Cx->CR1 |= (I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(I2C_CR1_PE);
	}
}


/*
 * Data Send and Receive
 */
void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (I2C_CR1_STOP);
}

//check flag
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t Flag_Name)
{
	if(pI2Cx->SR1 & Flag_Name){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);  //SlaveAddr is Slave address + r/nw bit = 0
	pI2Cx->DR = SlaveAddr;
}

void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;  //SlaveAddr is Slave address + r/nw bit = 1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//Check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (I2C_SR2_MSL))
	{
		//DEVICE IS IN MASTER MODE
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}else
		{
			//clear the ADDR flag ( read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}else
	{
		//DEVICE IS IN SLAVE MODE
	}
}

void I2C_ManageAcking(I2C_TypeDef *pI2Cx, uint8_t En0rDi)
{
	if(En0rDi == I2C_ACK_EN)
	{
		//enable the ack
		pI2Cx->CR1 |= (I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~(I2C_CR1_ACK);
	}
}

/**
  * @brief  Checks whether the last I2Cx Event is equal to the one passed
  *         as parameter.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  I2C_EVENT: specifies the event to be checked.
  *          This parameter can be one of the following values:
  *            @arg I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED: EV1
  *            @arg I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED: EV1
  *            @arg I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED: EV1
  *            @arg I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED: EV1
  *            @arg I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED: EV1
  *            @arg I2C_EVENT_SLAVE_BYTE_RECEIVED: EV2
  *            @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF): EV2
  *            @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL): EV2
  *            @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED: EV3
  *            @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF): EV3
  *            @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL): EV3
  *            @arg I2C_EVENT_SLAVE_ACK_FAILURE: EV3_2
  *            @arg I2C_EVENT_SLAVE_STOP_DETECTED: EV4
  *            @arg I2C_EVENT_MASTER_MODE_SELECT: EV5
  *            @arg I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: EV6
  *            @arg I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: EV6
  *            @arg I2C_EVENT_MASTER_BYTE_RECEIVED: EV7
  *            @arg I2C_EVENT_MASTER_BYTE_TRANSMITTING: EV8
  *            @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED: EV8_2
  *            @arg I2C_EVENT_MASTER_MODE_ADDRESS10: EV9
  *
  * @note   For detailed description of Events, please refer to section I2C_Events
  *         in stm32f4xx_i2c.h file.
  *
  * @retval An ErrorStatus enumeration value:
  *           - SUCCESS: Last event is equal to the I2C_EVENT
  *           - ERROR: Last event is different from the I2C_EVENT
  */

ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
  uint32_t lastevent = 0;
  uint32_t flag1 = 0, flag2 = 0;
  ErrorStatus status = ERROR;

  /* Read the I2Cx status register */
  flag1 = I2Cx->SR1;
  flag2 = I2Cx->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_MASK;

  /* Check whether the last event contains the I2C_EVENT */
  if ((lastevent & I2C_EVENT) == I2C_EVENT)
  {
    /* SUCCESS: last event is equal to I2C_EVENT */
    status = SUCCESS;
  }
  else
  {
    /* ERROR: last event is different from I2C_EVENT */
    status = ERROR;
  }
  /* Return status */
  return status;
}

/***************************************************************************
 ********************* Data Send and Receive********************************
 ***************************************************************************/

/**
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Data: Byte to be transmitted..
  * @retval None
  */
void I2C_SendData(I2C_TypeDef* I2Cx, uint8_t Data)
{
  /* Write in the DR register the data to be sent */
  I2Cx->DR = Data;
}

/**
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval The value of the received data.
  */
uint8_t I2C_ReceiveData(I2C_TypeDef* I2Cx)
{
  /* Return the data in the DR register */
  return (uint8_t)I2Cx->DR;
}

/***************************************************
 * @fn						-I2C_MasterSendData
 *
 * @brief  					-enable or disable I2C.
 *
 * @param[in]				- pointer to a I2C_Handle_t structure that contains
 *         				 		the configuration information for the specified I2C peripheral.
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate The START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* 2. confirm that start generation is completed by checking the SB flag in the SR1
	 * Note: Until SB is cleared SCL will be stretched (pulled to low)
	 */
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB)));

	// 3. Send the address of the slave with r/nw bit  set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR)));

	/* 5. clear the ADDr flag according to its software sequence
	 * Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	 */
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. send the data until Len becomes 0
	while(Len > 0)
	{
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE))); //wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	/*
	 * 7. when len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	 *    Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	 *    when BTF=1 SCL will be stretched (pulled to low)
	 */
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE)));

	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF)));

	// 8. Generate STOP condition and master need not to wait for the completion of stop condition
	// Note: generating STOP, automatically clears the BTF
	if (Sr==I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/***************************************************
 * @fn						-I2C_MasterRecieveData
 *
 * @brief  					-enable or disable I2C.
 *
 * @param[in]				- pointer to a I2C_Handle_t structure that contains
 *         				 		the configuration information for the specified I2C peripheral.
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */

void I2C_MasterRecieveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate The START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	/* 2. confirm that start generation is completed by checking the SB flag in the SR1
	 * Note: Until SB is cleared SCL will be stretched (pulled to low)
	 */
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB)));

	// 3. Send the address of the slave with r/nw bit  set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR)));


	// procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DI);

		//generate STOP condition
		if (Sr==I2C_DISABLE_SR)
		{
		  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE)));


		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;


	}

	// procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for (uint32_t i = Len ; i>0; i--)
		{
			//wait until RXNE becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE)));

			if(i==2)  //if last 2 bytes are remaining
			{
				//Disable ACKING
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DI);

				//generate StOP condition
				if (Sr==I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}
	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_EN);
	}
}


/*********************************************************************************************************
 *************************************** APIs INTERRUPTS *************************************************
 *********************************************************************************************************/
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. FIRST find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //there is bits already implemented we can't write there

	*(NVIC_PR_BASE_ADDR + (4*iprx)) |= (IRQPriority << shift_amount );

}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITERREN);

	}

	return busystate;

}


/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ITERREN);

	}

	return busystate;
}



void I2C_CloseRecieveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx-> CR2 &= ~(I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx-> CR2 &= ~(I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = 0;

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx-> CR2 &= ~(I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx-> CR2 &= ~(I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_EN)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_EN);
	}
}
/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Interrupt handling for different I2C events (refer SR1)

 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//SB flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx-> SR1 & (I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx-> SR1 & (I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx-> SR1 & (I2C_SR1_TXE))
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0)
				{
					//1. generate the STOP condition
					if (pI2CHandle->Sr==I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_AppEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{

		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3)
	{
		//STOPF flag is set
		//Clear the STOPF (i.e 1) read SR1 2) write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the application that STOP is detected
		I2C_AppEventCallback(pI2CHandle,I2C_EV_STOP);

	}

	temp3 = pI2CHandle->pI2Cx-> SR1 & (I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx-> SR2 & (I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					//1. load the data in to DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					//2. decrement the TxLen
					pI2CHandle->TxLen--;

					//3. Increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx-> SR1 & (I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx-> SR2 & (I2C_SR2_MSL))
		{
			//RXNE flag is set
			//We have to do the data reception
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

					pI2CHandle->RxLen--;
				}

				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						//clear the ack bit
						I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
					}

					//read DR
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;

				}

				if(pI2CHandle->RxLen == 0)
				{
					//close the I2C data reception and notify the application

					//1. generate the stop condition
					if (pI2CHandle->Sr==I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Close the I2C rx
					I2C_CloseRecieveData(pI2CHandle);

					//3. Notify the application
					I2C_AppEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
				}
			}
		}

	}



}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file


 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~(I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_AppEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_ARLO);
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		//Implement the code to notify the application about the error

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
	}

}


/*
 *Application call back
 */
__weak void I2C_AppEventCallback(I2C_Handle_t *pHandle, uint8_t event)
{

}


