/*
 * SX_main.c
 *
 *  Created on: Jan 14, 2021
 *      Author: Nour
 */

#include "SX_main.h"


/************************Description************************

	STM32F4				LORA
		PA7			-	MOSI -
		PA6			-	MISO -
		PA5			-	SCK -
		PA4			- 	NSS -
		PA3			-	DIO1 -
		PA2			-	DIO0 -
		PA1			-	RESET -
		3.3v		-	3.3v -
		GND			- 	GND-

************************************************/

void SX_Inits(void)
{


	/******************************************************************
		 * PA3			-	DIO1 -
		 * PA2			-	DIO0 -
		 * PA1			-	RESET -
		 * NOTE: may be NSS pin also
	**************************************************************/
		GPIO_Handle_t SX_DIOPins;

		SX_DIOPins.pGPIOx = GPIOA;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

		//DIO1
		SX_DIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
		GPIO_Init(&SX_DIOPins);

		//DIO0
		SX_DIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
		GPIO_Init(&SX_DIOPins);

		//RESET
		SX_DIOPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
		SX_DIOPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;

		GPIO_Init(&SX_DIOPins);
	/************************************************************
	* 							GPIO FOR SPI Init
	************************************************************/
	GPIO_Handle_t SX_SPIPins;

	SX_SPIPins.pGPIOx = GPIOA;
	SX_SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SX_SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SX_SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SX_SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SX_SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SPI1->SCK PIN
	SX_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SX_SPIPins);

	//SPI1->MOSI PIN
	SX_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SX_SPIPins);

	//SPI1->MISO PIN
	SX_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SX_SPIPins);

	//SPI1->NSS PIN
	SX_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SX_SPIPins);


	/************************************************************
	 * 							SPI Init
	 ************************************************************/
	SPI_Handle_t SX_hspi;

	SX_hspi.pSPIx = SPI1;
	SX_hspi.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SX_hspi.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SX_hspi.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SX_hspi.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SX_hspi.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SX_hspi.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SX_hspi.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SX_hspi);

}
