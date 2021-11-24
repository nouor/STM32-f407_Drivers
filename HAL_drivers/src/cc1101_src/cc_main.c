/*
 * CC_main.c
 *
 *  Created on: Jan 14, 2021
 *      Author: Nour
 */

#include "CC_main.h"


/************************Description************************

	STM32F4				LORA
		PA7			-	MOSI -
		PA6			-	MISO -
		PA5			-	SCK -
		PA4			- 	NSS -
		PA3			-	GD0 -
		PA2			-	GD2 -
		PA1			-	RESET -
		3.3v		-	3.3v -
		GND			- 	GND-

************************************************/

void CC_Inits(void)
{


	/******************************************************************
		 * PA3			-	GD0 -
		 * PA2			-	GD2 -
		 * PA1			-	RESET -
		 * NOTE: may be NSS pin also
	**************************************************************/
		GPIO_Handle_t CC_DIOPins;

		CC_DIOPins.pGPIOx = GPIOA;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

		//GD0
		CC_DIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
		GPIO_Init(&CC_DIOPins);

		//GD2
		CC_DIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
		GPIO_Init(&CC_DIOPins);

		//IRQ configuration
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI1,NVIC_IRQ_PRI0);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI1,ENABLE);

		/*
		//RESET
		CC_DIOPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		CC_DIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;

		GPIO_Init(&CC_DIOPins);

		*/
	/************************************************************
	* 							GPIO FOR SPI Init
	************************************************************/
	GPIO_Handle_t CC_SPIPins;

	CC_SPIPins.pGPIOx = GPIOA;
	CC_SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	CC_SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	CC_SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	CC_SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	CC_SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SPI1->SCK PIN
	CC_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&CC_SPIPins);

	//SPI1->MOSI PIN
	CC_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&CC_SPIPins);

	//SPI1->MISO PIN
	CC_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&CC_SPIPins);

	//SPI1->NSS PIN
	CC_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&CC_SPIPins);


	/************************************************************
	 * 							SPI Init
	 ************************************************************/
	SPI_Handle_t CC_hspi;

	CC_hspi.pSPIx = SPI1;
	CC_hspi.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	CC_hspi.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	CC_hspi.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	CC_hspi.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	CC_hspi.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	CC_hspi.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	CC_hspi.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&CC_hspi);

}
