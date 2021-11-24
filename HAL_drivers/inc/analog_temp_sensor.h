/*
 * temp_sensor.h
 *
 *  Created on: Jan 18, 2021
 *      Author: Noor
 */

#ifndef HAL_DRIVERS_INC_ANALOG_TEMP_SENSOR_H_
#define HAL_DRIVERS_INC_ANALOG_TEMP_SENSOR_H_

#include "adc.h"
#include "main.h"

void TempSensor_Init();
float GET_TEMPR() ;

#endif /* HAL_DRIVERS_INC_ANALOG_TEMP_SENSOR_H_ */
