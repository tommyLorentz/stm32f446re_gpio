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
	uint8_t	SPI_DeviceMode;			/* @GPIO_PIN_NUMBERS */
	uint8_t	SPI_BusConfig;			/* @GPIO_PIN_MODES */
	uint8_t	SPI_SclkSpeed;			/* @GPIO_PIN_SPEED */
	uint8_t	SPI_Dff;				/* @GPIO_INTERNAL_PUPD */
	uint8_t	SPI_Cpol;				/* @GPIO_OUTPUT_TYPE */
	uint8_t	SPI_Cpha;				/* @GPIO_OUTPUT_TYPE */
	uint8_t	SPI_Ssm;				/* @GPIO_OUTPUT_TYPE */

}SPI_PinConfig_t;

/*
 * This is a Handle structure for a SPI pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t	*pSpiBase;		/* This holds the base address of the SPI port to which the pin belongs */
	SPI_PinConfig_t SPI_PinConfig;	/* This holds SPI pin configuration settings */

}SPI_Handle_t;


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
void SPI_SendData(SPI_RegDef_t *pBase, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pBase, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration
 */
void SPI_IrqInteruptConfig(uint8_t IrqPosition, uint8_t IsEn);
void SPI_IrqPriorityConfig(uint8_t IrqPosition, uint8_t IrqPriority);

/*
 * ISR handling
 */
void SPI_IrqHandling(SPI_Handle_t *pSpiPinHandle);

/*
 * Other peripheral control APIs
 */



#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
