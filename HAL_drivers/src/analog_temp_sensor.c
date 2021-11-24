/*
 * temp_sensor.c
 *
 *  Created on: Jan 16, 2021
 *      Author: khlod
 */
#include "analog_temp_sensor.h"



void TempSensor_Init(){
	ADC_init();
	GPIO_Handle_t ADC_pins;
	ADC_pins.pGPIOx = GPIOA;
	ADC_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	ADC_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	ADC_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&ADC_pins);
}

float GET_TEMPR(){
	uint16_t adc_value=0;

	ADC1_startConversion();
	adc_value=ADC_DATA();

	//variable definitions
	float adc_resolution, v_out , tempr=0;
	uint16_t t;

	adc_resolution =(3.3/1024);
	v_out=adc_resolution *adc_value;
	//the output is 10milivolt per celesious degree
	tempr =(v_out/0.01);
	t=(uint16_t) tempr;
	return tempr;

}

