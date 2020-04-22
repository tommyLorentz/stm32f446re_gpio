/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 22, 2020
 *      Author: tommy
 */

#include <stm32f446xx_spi_driver.h>

/*
 * Peripheral clock setup
 */
/************************************************************
 * @fn          - SPI_PeriClockControl
 *
 * @brief		- This function enable and disables peripheral clock for the given SPI port
 *
 * @param[in]   - base address of the gpio peripheral
 * @param[in]   - ENABLE or DISABLE macros
 * @param[in]   -
 *
 * @return		- none
 *
 * @Note        - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pBase, uint8_t IsEn)
{
	if (IsEn == ENABLE)
	{
		if (pBase == SPI1) SPI1_PCLK_EN();
		else if (pBase == SPI2) SPI2_PCLK_EN();
		else if (pBase == SPI3) SPI3_PCLK_EN();
		else if (pBase == SPI4) SPI4_PCLK_EN();
	}
	else
	{
		if (pBase == SPI1) SPI1_PCLK_DI();
		else if (pBase == SPI2) SPI2_PCLK_DI();
		else if (pBase == SPI3) SPI3_PCLK_DI();
		else if (pBase == SPI4) SPI4_PCLK_DI();
	}
}

/*
 * Init and Deinit
 */
void SPI_Init(SPI_Handle_t *pSpiPinHandle)
{
	uint32_t reg = 0, temp;
	// Configure the SPI_CR1 register

	//  1. configure the device mode
	if (pSpiPinHandle->SPI_PinConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << 2);
	}
	else
	{
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << 2);
	}

	//  1. configure BIDIMODE: Bidirectional data mode enable
	if (pSpiPinHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		// uni-direction
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << 15);
	}
	else if (pSpiPinHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		// bi-direction
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << 15);
	}
	else if (pSpiPinHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		// uni-direction and rx-only
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << 15);
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << 10);
	}

}

void SPI_DeInit(SPI_Handle_t *pSpiPinHandle)
{
	if (pSpiPinHandle->pSpiBase == SPI1) SPI1_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI2) SPI2_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI3) SPI3_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI4) SPI4_REG_RESET();
}
