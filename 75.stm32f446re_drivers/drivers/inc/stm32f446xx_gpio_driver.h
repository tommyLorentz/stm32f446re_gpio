/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 2020年4月14日
 *      Author: tommy
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPupdControl;
	uint8_t	GPIO_PinOpType;
	uint8_t	GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t	*pGpioHandle;		/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings */

}GPIO_Handle_t;

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
