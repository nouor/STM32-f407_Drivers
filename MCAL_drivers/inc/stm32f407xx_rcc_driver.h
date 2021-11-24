/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Feb 4, 2021
 *      Author: Noor
 */

#ifndef MCAL_DRIVERS_INC_STM32F407XX_RCC_DRIVER_H_
#define MCAL_DRIVERS_INC_STM32F407XX_RCC_DRIVER_H_


#include "stm32f4xx.h"
#include "stm32f407xx.h"

void SysClockConfig (void);

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);


#endif /* MCAL_DRIVERS_INC_STM32F407XX_RCC_DRIVER_H_ */
