/**
  ******************************************************************************
  * @file    main.c
  * @author  Auto-generated by STM32CubeIDE
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_gpio_driver.h"
#include <string.h>
#include <stdio.h>
#include <sys/types.h>

// extern void initialise_monitor_handles();

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_CONSOLE_PRINT	0x53
#define COMMAND_ID_READ			0x54


#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

// arduino pin number 9
#define LED_PIN					9
#define LED_ON					1

void delay(void)
{
	uint32_t i;
	for (i=0; i< 1000; ++i) {}
}

uint8_t SPI_VerifyResponse(uint8_t code)
{
	if (code == 0xF5)
		return ACK;
	else
		return NACK;
}

void SPI_CommandTest(uint8_t Command, uint8_t *Arg, uint8_t ArgLen, uint8_t *RxData, uint8_t RxLen)
{
	uint8_t dummy_write[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	uint8_t dummy_read[10];
	uint8_t ack_byte, i;

	// send command
	SPI_SendData(SPI2, (uint8_t *) &Command, 1);
	// read dummy to clear RXNE
	SPI_ReceiveData(SPI2, (uint8_t *) &dummy_read, 1);

	// send dummy bit
	SPI_SendData(SPI2, (uint8_t *) &dummy_write, 1);
	// read the ack byte from slave
	SPI_ReceiveData(SPI2, (uint8_t *) &ack_byte, 1);

	delay();

	// Wait SPI until it's not busy
	if (SPI_VerifyResponse(ack_byte) == ACK)
	{
		SPI_SendData(SPI2, (uint8_t *) &Arg, ArgLen);

		if (RxLen)
		{
			// read dummy to clear RXNE
			SPI_ReceiveData(SPI2, (uint8_t *) &dummy_read, ArgLen);

			for (i=0; i<RxLen; ++i)
			{
				// send dummy bit
				SPI_SendData(SPI2, (uint8_t *) &dummy_write, 1);

				// read the ack byte from slave
				SPI_ReceiveData(SPI2, (uint8_t *) &RxData[i], 1);
			}
		}
	}
}

/*
 * PB12 --> SPI2_NSS
 * PB13 --> SPI2_SCK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * ALT: function mode: 5
 */
void SPI_GpioInit(void)
{
	GPIO_Handle_t gpioSettings;
	gpioSettings.pGpioBase = GPIOB;
	gpioSettings.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	//gpioSettings.GPIO_PinConfig.GPIO_PinSpeed
	gpioSettings.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PUPD_PU;
	gpioSettings.GPIO_PinConfig.GPIO_PinOpType = GPIO_OTYPE_PUPL;
	gpioSettings.GPIO_PinConfig.GPIO_PinAltFunMode = 5;

	// PB13 --> SPI2_SCK
	gpioSettings.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&gpioSettings);

	// PB15 --> SPI2_MOSI
	gpioSettings.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&gpioSettings);

	// PB14 --> SPI2_MISO
	gpioSettings.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&gpioSettings);

	// PB12 --> SPI2_NSS
	gpioSettings.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&gpioSettings);
}

void SPI2_Init(void)
{
	SPI_Handle_t spiSettings;
	spiSettings.pSpiBase = SPI2;
	spiSettings.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spiSettings.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	spiSettings.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spiSettings.SPI_PinConfig.SPI_Dff = SPI_DFF_8BITS;
	spiSettings.SPI_PinConfig.SPI_Cpol = SPI_CPOL_0;
	spiSettings.SPI_PinConfig.SPI_Cpha = SPI_CPHA_0;
	spiSettings.SPI_PinConfig.SPI_Ssm = SPI_SSM_DI; // software slave management enabled for NSS pin
	spiSettings.SPI_PinConfig.SPI_Lsbfirst = SPI_LSB_FIRST;
	SPI_Init(&spiSettings);
}

int main(void)
{
	// 26.3.7 SPI configuration
	// 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
	SPI_GpioInit();

	// 2. Write to the SPI_CR1 register
	SPI2_Init();

	// NSS output enable
	SPI_PeripheralEnableConfig(SPI2, ENABLE);


	uint8_t command = COMMAND_LED_CTRL;
	uint8_t arg[2];
	uint8_t rxData[10];

	command = COMMAND_LED_CTRL;
	arg[0] = LED_PIN;
	arg[1] = LED_ON;
	SPI_CommandTest(command, arg, 2, NULL, 0);

	command = COMMAND_SENSOR_READ;
	arg[0] = ANALOG_PIN0;
	SPI_CommandTest(command, arg, 1, rxData, 1);

	command = COMMAND_LED_READ;
	arg[0] = LED_PIN;
	SPI_CommandTest(command, arg, 1, rxData, 1);

	command = COMMAND_CONSOLE_PRINT;
	arg[0] = LED_PIN;
	SPI_CommandTest(command, arg, 1, NULL, 0);

	command = COMMAND_ID_READ;
	arg[0] = 0xff;
	SPI_CommandTest(command, arg, 1, rxData, 10);


	// Wait SPI until it's not busy
	while (SPI_GetFlagStatus(SPI2, SPI_SR_BSY_FLAG)) {}

	SPI_PeripheralEnableConfig(SPI2, DISABLE);
	for(;;);
}
