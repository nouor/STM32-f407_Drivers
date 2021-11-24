

#ifndef HAL_DRIVERS_INC_CC1101_INC_CC_SPI_H_
#define HAL_DRIVERS_INC_CC1101_INC_CC_SPI_H_

#include "stm32f407xx_spi_driver.h"

#include "cc1101.h"
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"

void SPI_CC_Cmd(uint8_t data);
uint8_t SPI_CC_Read(uint8_t adr);
uint8_t SPI_CC_Read8bit(void);
void SPI_CC_Write(uint8_t adr, uint8_t data);
void SPI_CC_BurstRead(uint8_t adr, uint8_t *p_data, uint8_t length);
void SPI_CC_BurstWrite(uint8_t adr, const uint8_t *p_data, uint8_t length);



#endif /* HAL_DRIVERS_INC_CC1101_INC_CC_SPI_H_ */
