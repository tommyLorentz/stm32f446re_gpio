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

}

void SPI_DeInit(SPI_Handle_t *pSpiPinHandle)
{
	if (pSpiPinHandle->pSpiBase == SPI1) SPI1_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI2) SPI2_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI3) SPI3_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI4) SPI4_REG_RESET();
}
