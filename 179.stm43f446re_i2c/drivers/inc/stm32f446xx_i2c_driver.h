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
	uint32_t	I2C_SclkSpeed;			/* @I2C_SclkSpeed */
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_AckControl;			/* @ */
	uint8_t		I2C_FmDutyCycle;		/* @ */

}I2C_PinConfig_t;

/*
 * This is a Handle structure for a I2C pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	I2C_RegDef_t	*pI2cBase;		/* This holds the base address of the I2C port to which the pin belongs */
	I2C_PinConfig_t I2C_PinConfig;	/* This holds I2C pin configuration settings */

}I2C_Handle_t;

/*
 * @I2C_SclkSpeed
 */
#define I2C_SCLK_SPEED_SM		100000
#define I2C_SCLK_SPEED_FM		400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FmDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1





#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
