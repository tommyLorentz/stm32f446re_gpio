/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Apr 27, 2020
 *      Author: tommy
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;			/* @SPI_DeviceMode */
	uint8_t	SPI_BusConfig;			/* @SPI_BusConfig */
	uint8_t	SPI_SclkSpeed;			/* @SPI_SclkSpeed */
	uint8_t	SPI_Dff;				/* @SPI_Dff */
	uint8_t	SPI_Cpol;				/* @SPI_Cpol */
	uint8_t	SPI_Cpha;				/* @SPI_Cpha */
	uint8_t	SPI_Ssm;				/* @SPI_Ssm */
	uint8_t	SPI_Lsbfirst;			/* @SPI_Lsbfirst */

}SPI_PinConfig_t;

/*
 * This is a Handle structure for a SPI pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t	*pSpiBase;		/* This holds the base address of the SPI port to which the pin belongs */
	SPI_PinConfig_t SPI_PinConfig;	/* This holds SPI pin configuration settings */
	uint8_t 			*pTxBuffer;
	uint8_t 			*pRxBuffer;
	int32_t 			TxLen;
	int32_t				RxLen;
	uint8_t				TxState;		/* @SPI_TxRxState */
	uint8_t 			RxState;		/* @SPI_TxRxState */

}SPI_Handle_t;

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
