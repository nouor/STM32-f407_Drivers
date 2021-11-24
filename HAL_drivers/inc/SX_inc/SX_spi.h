/*
 * SX_spi.h
 *
 *  Created on: Jan 14, 2021
 *      Author: Noor
 */

#ifndef HAL_DRIVERS_INC_SX_SPI_H_
#define HAL_DRIVERS_INC_SX_SPI_H_

#include "stm32f407xx_spi_driver.h"

void SPI_Cmd(uint8_t data);
uint8_t SPI_Read(uint8_t adr);
uint8_t SPI_Read8bit(void);
void SPI_Write(uint8_t adr, uint8_t data);
void SPI_BurstRead(uint8_t adr, uint8_t *p_data, uint8_t length);
void SPI_BurstWrite(uint8_t adr, const uint8_t *p_data, uint8_t length);


#endif /* HAL_DRIVERS_INC_SX_SPI_H_ */
