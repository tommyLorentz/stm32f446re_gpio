/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Apr 22, 2020
 *      Author: tommy
 */

#include <stm32f446xx_spi_driver.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSpiPinHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSpiPinHandle);
static void spi_overrun_interrupt_handle(SPI_Handle_t *pSpiPinHandle);

static void spi_txe_interrupt_handle(SPI_Handle_t *pSpiPinHandle)
{
	// check the DFF bit in data format
	if (pSpiPinHandle->pSpiBase->CR1 & (0x01 << SPI_CR1_DFF_OFFSET))
	{
		pSpiPinHandle->pSpiBase->DR = *(( uint16_t *) pSpiPinHandle->pTxBuffer);
		pSpiPinHandle->TxLen -= 2;
		( uint16_t *) pSpiPinHandle->pTxBuffer++;
	}else
	{
		pSpiPinHandle->pSpiBase->DR = *(( uint8_t *) pSpiPinHandle->pTxBuffer);
		pSpiPinHandle->TxLen -= 1;
		pSpiPinHandle->pTxBuffer++;
	}

	if (0 == pSpiPinHandle->TxLen)
	{
		// TxLen is 0, then close the SPI transmission and inform the application that Tx is over
		pSpiPinHandle->pSpiBase->CR2 &= ~(0x1 << SPI_CR2_TXEIE_OFFSET);
		pSpiPinHandle->pTxBuffer = NULL;
		pSpiPinHandle->TxLen = 0;
		pSpiPinHandle->TxState = SPI_STATE_READY;
		SPI_ApplicationEventCallback(pSpiPinHandle, SPI_EVENT_TX_COMPLETE);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSpiPinHandle)
{

}

static void spi_overrun_interrupt_handle(SPI_Handle_t *pSpiPinHandle)
{

}


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
		while(SPI_TXE_NEMPTY == SPI_GetFlagStatus(pBase, SPI_SR_TXE_FLAG)){}

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
	while(Len > 0)
	{
		// 1. wait until TXE buffer is empty
		while(SPI_RXNE_EMPTY == SPI_GetFlagStatus(pBase, SPI_SR_RXNE_FLAG)){}

		// check the DFF bit in data format
		if (pBase->CR1 & (0x01 << SPI_CR1_DFF_OFFSET))
		{
			*(( uint16_t *) pRxBuffer) = pBase->DR;
			Len -= 2;
			( uint16_t *) pRxBuffer++;
		}else
		{
			*(( uint8_t *) pRxBuffer) = pBase->DR;
			Len -= 1;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIt(SPI_Handle_t *pSpiPinHandle, uint8_t *pTxBuffer, int32_t Len)
{
	uint8_t state = pSpiPinHandle->TxState;
	if (state == SPI_STATE_BUSY_RX)
	{
		// 1. Save the Tx buffer address and len information
		pSpiPinHandle->pTxBuffer = pTxBuffer;
		pSpiPinHandle->TxLen = Len;

		// 2. Mark the SPI state as busy in transmission
		pSpiPinHandle->TxState = SPI_STATE_BUSY_TX;

		// 3. Enable the TXEIE control bit register
		pSpiPinHandle->pSpiBase->CR2 |= (0x01 << SPI_CR2_TXEIE_OFFSET);
	}

	state = pSpiPinHandle->TxState;
	return state;
}

uint8_t SPI_ReceiveDataIt(SPI_Handle_t *pSpiPinHandle, uint8_t *pRxBuffer, int32_t Len)
{
	uint8_t state = pSpiPinHandle->RxState;
	if (state == SPI_STATE_BUSY_TX)
	{
		// 1. Save the Tx buffer address and len information
		pSpiPinHandle->pRxBuffer = pRxBuffer;
		pSpiPinHandle->RxLen = Len;

		// 2. Mark the SPI state as busy in transmission
		pSpiPinHandle->RxState = SPI_STATE_BUSY_RX;

		// 3. Enable the TXEIE control bit register
		pSpiPinHandle->pSpiBase->CR2 |= (0x01 << SPI_CR2_RXNEIE_OFFSET);
	}

	state = pSpiPinHandle->RxState;
	return state;
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

/*
 * IRQ configuration
 */
void SPI_IrqInteruptConfig(uint8_t IrqPosition, uint8_t IsEn)
{
	__vo uint32_t *pNvicBase;

	// Set or clear interrupt register
	if (IsEn == ENABLE)
	{
		// program ISER register
		pNvicBase = (__vo uint32_t *) (NVIC_ISER_BASE + (IrqPosition / 32) * NVIC_GENERIC_OFFSET);
		*pNvicBase |=  ( 0x1 << (IrqPosition % 32));
	}
	else
	{
		// program ICER register
		pNvicBase = (__vo uint32_t *) (NVIC_ICER_BASE + (IrqPosition / 32) * NVIC_GENERIC_OFFSET);
		*pNvicBase |=  ( 0x1 << (IrqPosition % 32));
	}
}

/*
 * IRQ priority configuration
 */
void SPI_IrqPriorityConfig(uint8_t IrqPosition, uint8_t IrqPriority)
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
void SPI_IrqHandling(SPI_Handle_t *pSpiPinHandle)
{
	uint8_t temp1, temp2;
	// check for TXE flag
	temp1 = SPI_GetFlagStatus(pSpiPinHandle->pSpiBase, SPI_SR_TXE_FLAG);
	/* Tx buffer empty interrupt enable */
	temp2 = pSpiPinHandle->pSpiBase->CR2 & (0x1 << SPI_CR2_TXEIE_OFFSET);

	if (temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pSpiPinHandle);
	}

	// check for RXNE flag
	temp1 = SPI_GetFlagStatus(pSpiPinHandle->pSpiBase, SPI_SR_RXNE_FLAG);
	/* Rx buffer not empty interrupt enable */
	temp2 = pSpiPinHandle->pSpiBase->CR2 & (0x1 << SPI_CR2_RXNEIE_OFFSET);
	if (temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pSpiPinHandle);
	}

	// check for overrun flag
	temp1 = SPI_GetFlagStatus(pSpiPinHandle->pSpiBase, SPI_SR_OVR_FLAG);
	/* Tx buffer empty interrupt enable */
	temp2 = pSpiPinHandle->pSpiBase->CR2 & (0x1 << SPI_CR2_ERRIE_OFFSET);
	if (temp1 && temp2)
	{
		// handle overrun
		spi_overrun_interrupt_handle(pSpiPinHandle);
	}
}

