/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 22, 2020
 *      Author: tommy
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;			/* @GPIO_PIN_NUMBERS */
	uint8_t	SPI_BusConfig;			/* @GPIO_PIN_MODES */
	uint8_t	SPI_SclkSpeed;			/* @GPIO_PIN_SPEED */
	uint8_t	SPI_Dff;				/* @GPIO_INTERNAL_PUPD */
	uint8_t	SPI_Cpol;				/* @GPIO_OUTPUT_TYPE */
	uint8_t	SPI_Cpha;				/* @GPIO_OUTPUT_TYPE */
	uint8_t	SPI_Ssm;				/* @GPIO_OUTPUT_TYPE */

}SPI_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t	*pSpiBase;		/* This holds the base address of the SPI port to which the pin belongs */
	SPI_PinConfig_t SPI_PinConfig;	/* This holds SPI pin configuration settings */

}SPI_handle_t;

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
