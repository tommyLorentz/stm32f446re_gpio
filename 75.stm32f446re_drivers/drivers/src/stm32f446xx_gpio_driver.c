/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 2020年4月14日
 *      Author: tommy
 */

#include "stm32f446xx_gpio_driver.h"


/*
 * Peripheral clock setup
 */
/************************************************************
 * @fn          - GPIO_PeriClockControl
 *
 * @brief		- This function enable and disables peripheral clock for the given GPIO port
 *
 * @param[in]   - base address of the gpio peripheral
 * @param[in]   - ENABLE or DISABLE macros
 * @param[in]   -
 *
 * @return		- none
 *
 * @Note        - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pHandle, uint8_t IsEn)
{

}

/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_RegDef_t *pHandle)
{

}

void GPIO_DeInit(GPIO_RegDef_t *pHandle)
{

}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pHandle, uint8_t PinNumber)
{

}

uint32_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pHandle)
{

}

uint8_t GPIO_WriteToOutputPin(GPIO_RegDef_t *pHandle, uint8_t PinNumber, uint8_t Value)
{

}

uint8_t GPIO_WriteToOutputPort(GPIO_RegDef_t *pHandle, uint16_t Value)
{

}

uint8_t GPIO_ToggleOutputPort(GPIO_RegDef_t *pHandle, uint8_t PinNumber)
{

}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IrqConfig(uint8_t IrqNumber, uint8_t IrqPriority, uint8_t IsEn)
{

}
void GPIO_IrqHandling(uint8_t PinNumber)
{

}
