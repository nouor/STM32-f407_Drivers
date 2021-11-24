/*
 * digital_temp_sensor.h
 *
 *  Created on: Feb 6, 2021
 *      Author: khlod
 */

#ifndef HAL_DRIVERS_INC_DIGITAL_TEMP_SENSOR_H_
#define HAL_DRIVERS_INC_DIGITAL_TEMP_SENSOR_H_

#include "stm32f4xx.h"
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"


#define MY_ADDR					     0x61

/* LM75 defines */
#define LM75_ADDR                     0x90 // LM75 address

/* LM75 registers */
#define LM75_REG_TEMP                 0x00 // Temperature
#define LM75_REG_CONF                 0x01 // Configuration
#define LM75_REG_THYS                 0x02 // Hysteresis
#define LM75_REG_TOS                  0x03 // Overtemperature shutdown


void LM75_Init();

void LM75_WriteReg(uint8_t reg, uint16_t value);
uint16_t LM75_ReadReg(uint8_t reg);
uint8_t LM75_ReadConf(void);
void LM75_WriteConf(uint8_t value);

void LM75_Shutdown(FunctionalState newstate);
int16_t LM75_Temperature(void);

#endif /* HAL_DRIVERS_INC_DIGITAL_TEMP_SENSOR_H_ */
