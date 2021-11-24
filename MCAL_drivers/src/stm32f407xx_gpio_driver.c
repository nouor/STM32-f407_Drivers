/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on:  2021
 *      Author: Nour
 */


#include "stm32f407xx_gpio_driver.h"
#include "stm32f4xx.h"



/*
 * Peripheral Clock setup
 */
/***************************************************
 * @fn						-GPIO_PeriClockControl
 *
 * @brief  					-enable or disable peripheral clock
 *
 * @param[in]				-base address of the gpio peripheral
 * @param[in]               -ENABLE or DISABLE macros
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi)
{
		if(EnorDi == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}
		}else
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_DI();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_DI();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_DI();
			}

		}
}

/*
 * Init and De-init
 */


/***************************************************
 * @fn						-GPIO_INIT
 *
 * @brief  					-Initializes the GPIOx peripheral according to the specified parameters in the GPIO_InitStruct.
 *
 * @param[in]				- pointer to a GPIO_InitTypeDef structure that contains
 *         				 the configuration information for the specified GPIO peripheral.
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; //temp register

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	//1. CONFIGURE THE MODE OF GPIO PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//THE NON INTERRUPT MODE
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //multiply by 2 because 2 bits
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		//INTERRUPT MODE
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if  (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if  (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTCR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIOA_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp=0;

	//2. CONFIGURE THE SPEED
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;

	//3. CONFIGURE THE PUPD
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;

	//4. CONFIGURE THE OPTYPE
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		temp=0;
	}
	//5. CONFIGURE THE ALT
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//cofigure the alt fuction register
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));


	}

}

/***************************************************
 * @fn						-GPIO_DeINIT
 *
 * @brief  					-enable or disable peripheral clock
 *
 * @param[in]				-De-initializes the GPIOx peripheral registers to their default reset values.
 * @param[in]               -
 *
 *
 * @return 					- none
 *
 * @Note                    - By default, The GPIO pins are configured in input floating mode (except JTAG pins).
 *
 */

void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI  )
	{
		GPIOH_REG_RESET();
	}

}




/*
 * Data read and write
 */
/***************************************************
 * @fn						-GPIO_ReadFromInputPin
 *
 * @brief  					-Reads the specified input port pin.
 *
 * @param[in]				-GPIOx: where x can be (A..I) to select the GPIO peripheral for
 *                			STM32F40xx/41xx and STM32F427x/437x devices.
 * @param  				    GPIO_Pin: specifies the port bit to read.
 *         				    This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 *
 * @return 					- The input port pin value
 *
 * @Note                    - none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/***************************************************
 * @fn						-GPIO_ReadFromInputPort
 *
 * @brief  					-Reads the specified GPIO input data port.
 *
 * @param[in]				-GPIOx: where x can be (A..I) to select the GPIO peripheral for
 *                			STM32F40xx/41xx and STM32F427x/437x devices.
 *
 *
 * @return 					- GPIO input data port value.
 *
 * @Note                    - none
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx)
{
	uint16_t value;

	value = (uint16_t) pGPIOx->IDR ;

	return value;
}

/***************************************************
 * @fn						-GPIO_WriteToOutputPin
 *
 * @brief  					- Sets or clears the selected data port bit
 *
 * @param[in]				-GPIOx: where x can be (A..I) to select the GPIO peripheral for
 *                                  STM32F40xx/41xx and STM32F427x/437x devices.
 * @param  		             -PinNumber: specifies the port bit to be written.
 *                                   This parameter can be one of GPIO_Pin_x where x can be (0..15).
 * @param                    -Value: specifies the value to be written to the selected bit.
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***************************************************
 * @fn						-GPIO_WriteToOutputPort
 *
 * @brief  					-Writes data to the specified GPIO data port.
 *
 * @param[in]				-GPIOx: where x can be (A..I) to select the GPIO peripheral for
 *                                  STM32F40xx/41xx and STM32F427x/437x devices.
 *							--Value: specifies the value to be written to the selected port.
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***************************************************
 * @fn						-GPIO_ToggleOutputPin
 *
 * @brief  					-Toggles the specified GPIO pin..
 *
 * @param[in]				--GPIOx: where x can be (A..I) to select the GPIO peripheral for
 *                                  STM32F40xx/41xx and STM32F427x/437x devices.
 * @param  		             -PinNumber: specifies the port bit to be written.
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}


/*
 * IRQ Configuration and ISR handling
 */

/***************************************************
 * @fn						-GPIO_IRQInterruptConfig
 *
 * @brief  					-enable or disable gpio interrupt
 *
 * @param[in]				--
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn						-GPIO_IRQPriorityConfig
 *
 * @brief  					-configure priority of gpio interrupt
 *
 * @param[in]				--
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. FIRST find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //there is bits already implemented we can't write there

	*(NVIC_PR_BASE_ADDR + (4*iprx)) |= (IRQPriority << shift_amount );

}

/***************************************************
 * @fn						-GPIO_IRQHandling
 *
 * @brief  					-handle gpio interrupt
 *
 * @param[in]				--
 *
 *
 * @return 					- none
 *
 * @Note                    - none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}

}
