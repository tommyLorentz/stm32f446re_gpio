/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 22, 2020
 *      Author: tommy
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t	SPI_DeviceMode;			/* @SPI_DeviceMode */
	uint8_t	SPI_BusConfig;			/* @SPI_BusConfig */
	uint8_t	SPI_SclkSpeed;			/* @SPI_SclkSpeed */
	uint8_t	SPI_Dff;				/* @SPI_Dff */
	uint8_t	SPI_Cpol;				/* @SPI_Cpol */
	uint8_t	SPI_Cpha;				/* @SPI_Cpha */
	uint8_t	SPI_Ssm;				/* @SPI_Ssm */
	uint8_t	SPI_Lsbfirst;			/* @SPI_Lsbfirst */

}SPI_PinConfig_t;

/*
 * This is a Handle structure for a SPI pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t	*pSpiBase;		/* This holds the base address of the SPI port to which the pin belongs */
	SPI_PinConfig_t SPI_PinConfig;	/* This holds SPI pin configuration settings */
	uint8_t 			*pTxBuffer;
	uint8_t 			*pRxBuffer;
	uint32_t 			TxLen;
	uint32_t			RxLen;
	uint8_t			TxState;		/* @SPI_TxRxState */
	uint8_t 			RxState;		/* @SPI_TxRxState */

}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		0
#define SPI_DEVICE_MODE_SLAVE		1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX		1
#define SPI_BUS_CONFIG_HALF_DUPLEX		2
#define SPI_BUS_CONFIG_SIMPLEX_RX		3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_Dff
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_Cpol
 */
#define SPI_CPOL_0			0
#define SPI_CPOL_1			1

/*
 * @SPI_Cpha
 */
#define SPI_CPHA_0			0
#define SPI_CPHA_1			1

/*
 * @SPI_Ssm
 */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1

/*
 * @SPI_Lsbfirst
 */
#define SPI_MSB_FIRST		0
#define SPI_LSB_FIRST		1

/*
 * @SPI_TxRxState
 */
#define SPI_STATE_READY		0
#define SPI_STATE_BUSY_RX	1
#define SPI_STATE_BUSY_TX	2


/*
 * SPI related status flags definition
 */
#define SPI_TXE_FLAG		(0x1 << SPI_SR_TXE_OFFSET)
#define SPI_RXNE_FLAG		(0x1 << SPI_SR_RXNE_OFFSET)
#define SPI_BSY_FLAG		(0x1 << SPI_SR_BSY_OFFSET)

/*
 * Get SPI register status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pBase, uint32_t FlagName);

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pBase, uint8_t IsEn);

/*
 * Init and Deinit
 */
void SPI_Init(SPI_Handle_t *pSpiPinHandle);
void SPI_DeInit(SPI_Handle_t *pSpiPinHandle);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pBase, uint8_t *pTxBuffer, int32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pBase, uint8_t *pRxBuffer, int32_t Len);

uint8_t SPI_SendDataIt(SPI_Handle_t *pSpiPinHandle, uint8_t *pTxBuffer, int32_t Len);
uint8_t SPI_ReceiveDataIt(SPI_Handle_t *pSpiPinHandle, uint8_t *pRxBuffer, int32_t Len);

/*
 * IRQ configuration
 */
void SPI_IrqInteruptConfig(uint8_t IrqPosition, uint8_t IsEn);
void SPI_IrqPriorityConfig(uint8_t IrqPosition, uint8_t IrqPriority);

/*
 * ISR handling
 */
void SPI_IrqHandling(uint8_t PinNumber);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralEnableConfig(SPI_RegDef_t *pBase, uint8_t IsEn);
void SPI_SsiConfig(SPI_RegDef_t *pBase, uint8_t IsEn);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
