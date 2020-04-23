/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 22, 2020
 *      Author: tommy
 */

#include <stm32f446xx_spi_driver.h>
/*
 * Get SPI register status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pBase, uint32_t FlagName)
{
	if (pBase->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

/*
 * @fn          - SPI_PeriClockControl
 *
 * @brief		- This function enable and disables peripheral clock for the given SPI port
 *
 * @param[in]   - base address of the spi peripheral
 * @param[in]   - ENABLE or DISABLE macros
 * @param[in]   -
 *
 * @return		- none
 *
 * @Note        - Blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pBase, uint8_t *pTxBuffer, int32_t Len)
{
	while(Len > 0)
	{
		// 1. wait until TXE buffer is empty
		while(SPI_TXE_NEMPTY == SPI_GetFlagStatus(pBase, SPI_TXE_FLAG)){}

		// check the DFF bit in data format
		if (pBase->CR1 & (0x01 << SPI_CR1_DFF_OFFSET))
		{
			pBase->DR = *(( uint16_t *) pTxBuffer);
			Len -= 2;
			( uint16_t *) pTxBuffer++;
		}else
		{
			pBase->DR = *(( uint8_t *) pTxBuffer);
			Len -= 1;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pBase, uint8_t *pRxBuffer, int32_t Len)
{

}


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

	// Enable peripheral clock
	SPI_PeriClockControl(pSpiPinHandle->pSpiBase, ENABLE);

	// Configure the SPI_CR1 register
	// a) Configure the serial clock baud rate using the BR[2:0] bits
	temp = pSpiPinHandle->SPI_PinConfig.SPI_SclkSpeed & (0x07 << SPI_CR1_BR_OFFSET);
	reg = pSpiPinHandle->pSpiBase->CR1 & ~(0x07 << SPI_CR1_BR_OFFSET);
	pSpiPinHandle->pSpiBase->CR1 = temp | reg;


	// b) Configure the CPOL and CPHA bits
	if (pSpiPinHandle->SPI_PinConfig.SPI_Cpol == SPI_CPOL_1)
	{
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_CPOL_OFFSET);
	}
	else
	{
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_CPOL_OFFSET);
	}

	// b) Configure the CPOL and CPHA bits
	if (pSpiPinHandle->SPI_PinConfig.SPI_Cpha == SPI_CPHA_1)
	{
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_CPHA_OFFSET);
	}
	else
	{
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_CPHA_OFFSET);
	}

	// c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and BIDIOE
	if (pSpiPinHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		// uni-direction
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_BIDIMODE_OFFSET);
	}
	else if (pSpiPinHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		// bi-direction
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_BIDIMODE_OFFSET);
	}
	else if (pSpiPinHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	{
		// uni-direction and rx-only
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_BIDIMODE_OFFSET);
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_RXONLY_OFFSET);
	}

	// d) Configure the LSBFIRST bit to define the frame format
	if (pSpiPinHandle->SPI_PinConfig.SPI_Lsbfirst == SPI_LSB_FIRST)
	{
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_LSBFIRST_OFFSET);
	}
	else
	{
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_LSBFIRST_OFFSET);
	}

	// e) Configure the CRCEN and CRCEN bits if CRC is needed

	// f) Configure SSM and SSI
	if (pSpiPinHandle->SPI_PinConfig.SPI_Ssm == SPI_SSM_EN)
	{
		// Software slave management enabled
		// Pull NSS pin by internal signal
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_SSM_OFFSET);
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_SSI_OFFSET);
	}
	else
	{
		// Software slave management disabled
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_SSM_OFFSET);
		pSpiPinHandle->pSpiBase->CR2 |= (0x01 << SPI_CR2_SSOE_OFFSET);
	}


	//  1. configure the device mode
	if (pSpiPinHandle->SPI_PinConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		pSpiPinHandle->pSpiBase->CR1 |= (0x01 << SPI_CR1_MSTR_OFFSET);
	}
	else
	{
		pSpiPinHandle->pSpiBase->CR1 &= ~(0x01 << SPI_CR1_MSTR_OFFSET);
	}
}

void SPI_DeInit(SPI_Handle_t *pSpiPinHandle)
{
	if (pSpiPinHandle->pSpiBase == SPI1) SPI1_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI2) SPI2_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI3) SPI3_REG_RESET();
	else if (pSpiPinHandle->pSpiBase == SPI4) SPI4_REG_RESET();
}

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralEnableConfig(SPI_RegDef_t *pBase, uint8_t IsEn)
{
	if (IsEn == ENABLE)
	{
		pBase->CR1 |= (0x01 << SPI_CR1_SPE_OFFSET);
	}
	else
	{
		pBase->CR1 &= ~(0x01 << SPI_CR1_SPE_OFFSET);
	}
}


/*
 * Other peripheral control APIs
 */
void SPI_SsiConfig(SPI_RegDef_t *pBase, uint8_t IsEn)
{
	if (IsEn == ENABLE)
	{
		pBase->CR1 |= (0x01 << SPI_CR1_SSI_OFFSET);
	}else
	{
		pBase->CR1 &= ~(0x01 << SPI_CR1_SSI_OFFSET);
	}
}
