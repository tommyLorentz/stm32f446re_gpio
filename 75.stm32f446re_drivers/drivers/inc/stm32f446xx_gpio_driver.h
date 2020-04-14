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
	uint8_t	GPIO_PinNumber;			/* @GPIO_PIN_NUMBERS */
	uint8_t	GPIO_PinMode;			/* @GPIO_PIN_MODES */
	uint8_t	GPIO_PinSpeed;			/* @GPIO_PIN_SPEED */
	uint8_t	GPIO_PinPupdControl;	/* @GPIO_INTERNAL_PUPD */
	uint8_t	GPIO_PinOpType;			/* @GPIO_OUTPUT_TYPE */
	uint8_t	GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t	*pGpioBase;		/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings */

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO ping numbers
 *
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO ping possible modes
 *
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALT			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_FTRT		6

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO port output type
 */
#define GPIO_OTYPE_PUPL			0
#define GPIO_OTYPE_OPENDRAIN	1

/*
 * @GPIO_PIN_SPEED
 * GPIO port output speed
 */
#define GPIO_OSPEED_LOW			0
#define GPIO_OSPEED_MEDIUM		1
#define GPIO_OSPEED_FAST		2
#define GPIO_OSPEED_HIGH		3

/*
 * @GPIO_INTERNAL_PUPD
 * GPIO port pull-up/pull-down
 */
#define GPIO_PUPD_NONE			0
#define GPIO_PUPD_PU			1
#define GPIO_PUPD_PD			2


/**************************************************
 *
 *                 APIs supported by the driver
 *
 **************************************************/
/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pHandle, uint8_t IsEn);

/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGpioPinHandle);
void GPIO_DeInit(GPIO_Handle_t *pGpioPinHandle);

/*
 * Data input port read
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGpioBase, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGpioBase);

/*
 * Data output port read and write
 */
uint8_t GPIO_ReadfromOutputPin(GPIO_RegDef_t *pGpioBase, uint8_t PinNumber);
uint16_t GPIO_ReadfromOutputPort(GPIO_RegDef_t *pGpioBase);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGpioBase, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGpioBase, uint16_t Value);
void GPIO_ToggleOutputPort(GPIO_RegDef_t *pGpioBase, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IrqConfig(uint8_t IrqNumber, uint8_t IrqPriority, uint8_t IsEn);
void GPIO_IrqHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
