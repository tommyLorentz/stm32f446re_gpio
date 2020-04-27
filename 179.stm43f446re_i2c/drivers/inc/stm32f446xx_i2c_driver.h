/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Apr 27, 2020
 *      Author: tommy
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint32_t	I2C_SclkSpeed;			/* @I2C_SclkSpeed */
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_AckControl;			/* @ */
	uint8_t		I2C_FmDutyCycle;		/* @ */

}I2C_PinConfig_t;

/*
 * This is a Handle structure for a I2C pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO peripheral
	I2C_RegDef_t	*pI2cBase;		/* This holds the base address of the I2C port to which the pin belongs */
	I2C_PinConfig_t I2C_PinConfig;	/* This holds I2C pin configuration settings */

}I2C_Handle_t;

/*
 * @I2C_SclkSpeed
 */
#define I2C_SCLK_SPEED_SM		100000
#define I2C_SCLK_SPEED_FM		400000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FmDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


/*
 * I2C_SR1 related status flags definition
 */
#define I2C_SR1_SMBALERT_FLAG()		(0x01 << I2C_SR1_SMBALERT_OFFSET)	/* SMBus alert */
#define I2C_SR1_TIMEOUT_FLAG()		(0x01 << I2C_SR1_TIMEOUT_OFFSET)	/* Timeout or Tlow error  */
#define I2C_SR1_PECERR_FLAG()		(0x01 << I2C_SR1_PECERR_OFFSET)	/* PEC Error in reception */
#define I2C_SR1_OVR_FLAG()			(0x01 << I2C_SR1_OVR_OFFSET)	/* Overrun/Underrun  */
#define I2C_SR1_AF_FLAG()			(0x01 << I2C_SR1_AF_OFFSET)		/* Acknowledge failure */
#define I2C_SR1_ARLO_FLAG()			(0x01 << I2C_SR1_ARLO_OFFSET)	/* Arbitration lost (master mode) */
#define I2C_SR1_BERR_FLAG()			(0x01 << I2C_SR1_BERR_OFFSET)	/* Bus error */
#define I2C_SR1_TXE_FLAG()			(0x01 << I2C_SR1_TXE_OFFSET)	/* Data register empty (transmitters) */
#define I2C_SR1_RXNE_FLAG()			(0x01 << I2C_SR1_RXNE_OFFSET)	/* Data register not empty (receivers) */
#define I2C_SR1_STOPF_FLAG()		(0x01 << I2C_SR1_STOPF_OFFSET)	/* Stop detection (slave mode) */
#define I2C_SR1_ADD10_FLAG()		(0x01 << I2C_SR1_ADD10_OFFSET)	/* 10-bit header sent (Master mode) */
#define I2C_SR1_BTF_FLAG()			(0x01 << I2C_SR1_BTF_OFFSET)	/* Byte transfer finished */
#define I2C_SR1_ADDR_FLAG()			(0x01 << I2C_SR1_ADDR_OFFSET)	/* Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_SB_FLAG()			(0x01 << I2C_SR1_SB_OFFSET)		/* Start bit (Master mode) */

/*
 * I2C_SR2 related status flags definition
 */
#define I2C_SR2_PEC_FLAG()			(0xFF << I2C_SR2_PEC_OFFSET)	/* PEC[7:0] Packet error checking register */
#define I2C_SR2_DUALF_FLAG()		(0x01 << I2C_SR2_DUALF_OFFSET)	/* Dual flag (Slave mode) */
#define I2C_SR2_SMBHOST_FLAG()		(0x01 << I2C_SR2_SMBHOST_OFFSET)	/* SMBus host header (Slave mode) */
#define I2C_SR2_SMBDEFAULT_FLAG()	(0x01 << I2C_SR2_SMBDEFAULT_OFFSET)	/* SMBus device default address (Slave mode) */
#define I2C_SR2_GENCALL_FLAG()		(0x01 << I2C_SR2_GENCALL_OFFSET)	/* General call address (Slave mode) */
#define I2C_SR2_TRA_FLAG()			(0x01 << I2C_SR2_TRA_OFFSET)	/* Transmitter/receiver */
#define I2C_SR2_BUSY_FLAG()			(0x01 << I2C_SR2_BUSY_OFFSET)	/* Bus busy */
#define I2C_SR2_MSL_FLAG()			(0x01 << I2C_SR2_MSL_OFFSET)	/* Master/slave */

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pBase, uint8_t IsEn);

/*
 * Init and Deinit
 */
void I2C_Init(I2C_Handle_t *pI2CPinHandle);
void I2C_DeInit(I2C_Handle_t *pI2CPinHandle);

uint32_t RCC_GetPclk1Value(void);

/*
 * Data Send and Receive
 */


/*
 * IRQ configuration
 */
void I2C_IrqInteruptConfig(uint8_t IrqPosition, uint8_t IsEn);
void I2C_IrqPriorityConfig(uint8_t IrqPosition, uint8_t IrqPriority);

/*
 * ISR handling
 */


/*
 * Application Callback
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CPinHandle, uint8_t AppEvent);

/*
 * Other peripheral control APIs
 */
void I2C_PeripheralEnableConfig(I2C_RegDef_t *pBase, uint8_t IsEn);

/*
 * Get I2C register status
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pBase, uint8_t IsAddrOffset, uint32_t FlagName);



#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
