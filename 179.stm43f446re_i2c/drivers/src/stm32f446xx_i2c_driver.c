/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Apr 27, 2020
 *      Author: tommy
 */


#include "stm32f446xx_i2c_driver.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t	I2C_DeviceMode;			/* @I2C_DeviceMode */
	uint8_t	I2C_BusConfig;			/* @I2C_BusConfig */
	uint8_t	I2C_SclkSpeed;			/* @I2C_SclkSpeed */
	uint8_t	I2C_Dff;				/* @I2C_Dff */
	uint8_t	I2C_Cpol;				/* @I2C_Cpol */
	uint8_t	I2C_Cpha;				/* @I2C_Cpha */
	uint8_t	I2C_Ssm;				/* @I2C_Ssm */
	uint8_t	I2C_Lsbfirst;			/* @I2C_Lsbfirst */

}I2C_PinConfig_t;

/*
 * This is a Handle structure for a I2C pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	I2C_RegDef_t	*pI2cBase;		/* This holds the base address of the I2C port to which the pin belongs */
	I2C_PinConfig_t I2C_PinConfig;	/* This holds I2C pin configuration settings */
	uint8_t 			*pTxBuffer;
	uint8_t 			*pRxBuffer;
	int32_t 			TxLen;
	int32_t				RxLen;
	uint8_t				TxState;		/* @I2C_TxRxState */
	uint8_t 			RxState;		/* @I2C_TxRxState */

}I2C_Handle_t;
