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
#include "string.h"
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
	int8_t user_data[] = "Hello world";

	// 26.3.7 SPI configuration
	// 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
	SPI_GpioInit();

	// 2. Write to the SPI_CR1 register
	SPI2_Init();

	// NSS output enable
	SPI_PeripheralEnableConfig(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));

	// Wait SPI until it's not busy
	while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)) {}

	SPI_PeripheralEnableConfig(SPI2, DISABLE);
	for(;;);
}
