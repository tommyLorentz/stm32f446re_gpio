/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Apr 27, 2020
 *      Author: tommy
 */


#include "stm32f446xx_i2c_driver.h"

uint32_t RCC_GetPclk1Value(void)
{
	uint32_t clk;
	uint32_t clksrc;
	uint32_t HPRE_DIV, APB1_DIV;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	// Use HSI as system
	if (clksrc == 0)
	{
		clk = 16 * 1000 * 1000;
	}
	else
	{
		return 0;
	}

	HPRE_DIV = ((RCC->CFGR >> 4) & 0x0f);
	if (HPRE_DIV>= 8)
	{
		if(HPRE_DIV>= 12)
			clk >>= (HPRE_DIV-6);
		else
			clk >>= (HPRE_DIV-7);
	}

	APB1_DIV = ((RCC->CFGR >> 10) & 0x07);
	if (APB1_DIV>= 4)
	{
		clk >>= (APB1_DIV-3);
	}

	return clk;
}

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pBase, uint8_t IsEn)
{
	if (IsEn == ENABLE)
	{
		if (pBase == I2C1) I2C1_PCLK_EN();
		else if (pBase == I2C2) I2C2_PCLK_EN();
		else if (pBase == I2C3) I2C3_PCLK_EN();
	}
	else
	{
		if (pBase == I2C1) I2C1_PCLK_DI();
		else if (pBase == I2C2) I2C2_PCLK_DI();
		else if (pBase == I2C3) I2C3_PCLK_DI();
	}
}

/*
 * Init and Deinit
 */
void I2C_Init(I2C_Handle_t *pI2CPinHandle)
{
	uint32_t temp;

	// enable ack
	pI2CPinHandle->pI2cBase->CR1 |= (0x01 << I2C_CR1_ACK_OFFSET);

}

void I2C_DeInit(I2C_Handle_t *pI2CPinHandle)
{
	I2C_RegDef_t	*pI2cBase = pI2CPinHandle->pI2cBase;
	if (pI2cBase == I2C1) I2C1_REG_RESET();
	else if (pI2cBase == I2C2) I2C2_REG_RESET();
	else if (pI2cBase == I2C3) I2C3_REG_RESET();
}

/*
 * IRQ configuration
 */
void I2C_IrqInteruptConfig(uint8_t IrqPosition, uint8_t IsEn)
{

}

void I2C_IrqPriorityConfig(uint8_t IrqPosition, uint8_t IrqPriority)
{
	__vo uint8_t *pNvicBase;

	// Set priority register
	{
		// program ISER register
		pNvicBase = (__vo uint8_t *) (NVIC_PRI0 + IrqPosition);
		*pNvicBase = (IrqPriority << NVIC_PRI_BIT_SHIFT); /* only use 4 bit: 16 orders */
	}
}

/*
 * ISR handling
 */


/*
 * Other peripheral control APIs
 */
void I2C_PeripheralEnableConfig(I2C_RegDef_t *pBase, uint8_t IsEn)
{
	if (IsEn == ENABLE)
	{
		pBase->CR1 |= (0x01 << I2C_CR1_PE_OFFSET);
	}
	else
	{
		pBase->CR1 &= ~(0x01 << I2C_CR1_PE_OFFSET);
	}
}

/*
 * Get I2C register status
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pBase, uint8_t IsAddrOffset, uint32_t FlagName)
{
	if (IsAddrOffset == FALSE)
	{
		return pBase->SR1 & FlagName;
	}
	else
	{
		return pBase->SR2 & FlagName;
	}

}

