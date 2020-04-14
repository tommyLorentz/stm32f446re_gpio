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
	if (IsEn == ENABLE)
	{
		if (pHandle == GPIOA) GPIOA_PCLK_ENABLE();
		else if (pHandle == GPIOB) GPIOB_PCLK_ENABLE();
		else if (pHandle == GPIOC) GPIOC_PCLK_ENABLE();
		else if (pHandle == GPIOD) GPIOD_PCLK_ENABLE();
		else if (pHandle == GPIOE) GPIOE_PCLK_ENABLE();
		else if (pHandle == GPIOF) GPIOF_PCLK_ENABLE();
		else if (pHandle == GPIOG) GPIOG_PCLK_ENABLE();
		else if (pHandle == GPIOH) GPIOH_PCLK_ENABLE();
	}
	else
	{
		if (pHandle == GPIOA) GPIOA_PCLK_DI();
		else if (pHandle == GPIOB) GPIOB_PCLK_DI();
		else if (pHandle == GPIOC) GPIOC_PCLK_DI();
		else if (pHandle == GPIOD) GPIOD_PCLK_DI();
		else if (pHandle == GPIOE) GPIOE_PCLK_DI();
		else if (pHandle == GPIOF) GPIOF_PCLK_DI();
		else if (pHandle == GPIOG) GPIOG_PCLK_DI();
		else if (pHandle == GPIOH) GPIOH_PCLK_DI();
	}
}

/*
 * Init and Deinit
 */
void GPIO_Init(GPIO_Handle_t *pGpioPinHandle)
{
	uint32_t reg = 0, temp;

	// 1. configure the mode of gpio pin
	if (pGpioPinHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		reg = pGpioPinHandle->GPIO_PinConfig.GPIO_PinMode;
		reg <<= (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
		temp = ~( 0x3 << (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber * 2) ) & pGpioPinHandle->pGpioBase->MODER;
		pGpioPinHandle->pGpioBase->MODER = temp | reg;
	}
	else
	{
		// interrupt mode
	}

	// 2. configure the speed
	if (pGpioPinHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT)
	{
		reg = pGpioPinHandle->GPIO_PinConfig.GPIO_PinSpeed;
		reg <<= (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
		temp = ~( 0x3 << (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber * 2) ) & pGpioPinHandle->pGpioBase->OSPEEDER;
		pGpioPinHandle->pGpioBase->OSPEEDER = temp | reg;
	}

	// 3. configure the pupd settings
	{
		reg = pGpioPinHandle->GPIO_PinConfig.GPIO_PinPupdControl;
		reg <<= (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber * 2);
		temp = ~( 0x3 << (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber * 2) ) & pGpioPinHandle->pGpioBase->PUPDR;
		pGpioPinHandle->pGpioBase->PUPDR = temp | reg;
	}

	// 4. configure the output type
	if (pGpioPinHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUTPUT)
	{
		reg = pGpioPinHandle->GPIO_PinConfig.GPIO_PinOpType;
		reg <<= (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber);
		temp = ~( 0x1 << (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber) ) & pGpioPinHandle->pGpioBase->OTYPER;
		pGpioPinHandle->pGpioBase->OTYPER = temp | reg;
	}

	// 5. configure the alt functionality
	if (pGpioPinHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT)
	{
		uint32_t isHigh, offset;

		offset = (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber % 8) * 4;
		isHigh = (pGpioPinHandle->GPIO_PinConfig.GPIO_PinNumber / 8);

		reg = pGpioPinHandle->GPIO_PinConfig.GPIO_PinAltFunMode << offset; // offset for pin x
		temp = ~( 0xf << offset ) & pGpioPinHandle->pGpioBase->AFR[isHigh];
		pGpioPinHandle->pGpioBase->AFR[isHigh] = temp | reg;
	}
}

void GPIO_DeInit(GPIO_Handle_t *pGpioPinHandle)
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
