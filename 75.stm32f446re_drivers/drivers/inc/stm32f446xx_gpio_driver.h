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
void GPIO_Init(GPIO_RegDef_t *pHandle);
void GPIO_DeInit(GPIO_RegDef_t *pHandle);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pHandle, uint8_t PinNumber);
uint32_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pHandle);
uint8_t GPIO_WriteToOutputPin(GPIO_RegDef_t *pHandle, uint8_t PinNumber, uint8_t Value);
uint8_t GPIO_WriteToOutputPort(GPIO_RegDef_t *pHandle, uint16_t Value);
uint8_t GPIO_ToggleOutputPort(GPIO_RegDef_t *pHandle, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IrqConfig(uint8_t IrqNumber, uint8_t IrqPriority, uint8_t IsEn);
void GPIO_IrqHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
