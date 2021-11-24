/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 2, 2021
 *      Author: Nour
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

/*
 * stm32f401xe_gpio_driver.h
 *
 *  Created on: Jun 29, 2020
 *      Author: Noor
 */

#ifndef INC_STM32F401XE_GPIO_DRIVER_H_
#define INC_STM32F401XE_GPIO_DRIVER_H_


#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"



/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber; 					/*@GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;						/*@GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;						/*@GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;				/*@GPIO_PULLUP_PULLDOWN*/
	uint8_t GPIO_PinOPType;						/*@GPIO_OP_TYPE*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_TypeDef *pGPIOx;                  /*This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;		/*This holds GPIO pin configuration settings */

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_NO_0              	 0
#define GPIO_PIN_NO_1                1
#define GPIO_PIN_NO_2                2
#define GPIO_PIN_NO_3                3
#define GPIO_PIN_NO_4                4
#define GPIO_PIN_NO_5                5
#define GPIO_PIN_NO_6                6
#define GPIO_PIN_NO_7                7
#define GPIO_PIN_NO_8                8
#define GPIO_PIN_NO_9                9
#define GPIO_PIN_NO_10               10
#define GPIO_PIN_NO_11               11
#define GPIO_PIN_NO_12               12
#define GPIO_PIN_NO_13               13
#define GPIO_PIN_NO_14               14
#define GPIO_PIN_NO_15               15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN                  0
#define GPIO_MODE_OUT                 1
#define GPIO_MODE_ALTFN               2 //alternate_function
//#define GPIO_MODE_ANALOG              3
#define GPIO_MODE_IT_FT               4 //falling edge
#define GPIO_MODE_IT_RT               5 //rising edge
#define GPIO_MODE_IT_RFT              6 //falling-falling edge

/*
 * @GPIO_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP               0 //push-pull
#define GPIO_OP_TYPE_OD               1 //open_drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
//#define GPIO_SPEED_LOW				 	  0
//#define GPIO_SPEED_MEDIUM				  1
//#define GPIO_SPEED_HIGH            		  2
#define GPIO_SPEED_VERY_HIGH              3

/*
 *@GPIO_PULLUP_PULLDOWN
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2

/****************************************************************************************
 * 								APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 *****************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F401XE_GPIO_DRIVER_H_ */


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
