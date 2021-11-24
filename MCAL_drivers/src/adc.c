/*
 * adc.c
 *
 *  Created on: Dec 31, 2020
 *      Author: kholoud
 */

#include "adc.h"

ADC_HandleTypeDef hadc1;

void ADC_init(void)
{
	hadc1.Instance = ADC1;

	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
	ADC123_COMMON->CCR &= ~(ADC_CCR_ADCPRE);
	ADC123_COMMON->CCR |= ADC_CLOCK_SYNC_PCLK_DIV8;

	ADC1->CR1 &= ~(ADC_CR1_SCAN);

	ADC1->CR1 &= ~(ADC_CR1_RES);
	ADC1->CR1 |= ADC_RESOLUTION_10B;

	ADC1->CR2 &= ~(ADC_CR2_ALIGN);
	ADC1->CR2 |= ADC_DATAALIGN_RIGHT;

	ADC1->CR2 &= ~(ADC_CR2_EXTSEL);
	ADC1->CR2 &= ~(ADC_CR2_EXTEN);

	ADC1->CR2 &= ~(ADC_CR2_CONT);
	ADC1->CR1 &= ~(ADC_CR1_DISCEN);

	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->CR2 &= ~(ADC_CR2_DDS);

	ADC1->CR2 &= ~(ADC_CR2_EOCS);
	ADC1->CR2 |= (ADC_CR2_EOCS);

	ADC1->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, ADC_CHANNEL_1);
	ADC1->SMPR2 |= ADC_SMPR2(ADC_SAMPLETIME_480CYCLES, ADC_CHANNEL_1);

	/* Clear the old SQx bits for the selected rank */
	ADC1->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, 1);

	/* Set the SQx bits for the selected rank */
	ADC1->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_1, 1);


}


void ADC1_startConversion(void)
{
	uint32_t counter;
	//Enable ADC /*we need to know the voltage*/
	ADC1->CR2 |= ADC_CR2_ADON; //ADON BIT(1<<1)
	counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
	while(counter != 0U)
	{
		counter--;
	}
	//start conversion
	ADC1->CR2 |= ADC_CR2_SWSTART; //SWST
	//wait until end of conversion
	while(HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK);

}
uint16_t ADC_DATA(void){
	//get data

	return ADC1->DR;
}
