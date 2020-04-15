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

#include <string.h>

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

// #define VIDEO_103	"LED_PUSH_PULL"
// #define VIDEO_104	"LED_OPEN_DRAIN"
// #define VIDEO_105		  "BUTTOM_LED_PUSH_PULL"
#define VIDEO_115		  "PUSH_BUTTOM_INTERRUPT"

#if defined(VIDEO_103)
void delay(void)
{
	uint32_t i;
	for (i=0; i<100000; ++i) { }
}

void video103_push_pull_led(void)
{
	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0x00, sizeof(GpioLed));
	GpioLed.pGpioBase = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PUPD_NONE;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OTYPE_PUPL;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);


	while(1)
	{
		GPIO_ToggleOutputPin(GpioLed.pGpioBase, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}
}

#elif defined(VIDEO_104)
void delay(void)
{
	uint32_t i;
	for (i=0; i<100000; ++i) { }
}

void video104_open_drain_led(void)
{
	GPIO_Handle_t GpioLed;
	memset(&GpioLed, 0x00, sizeof(GpioLed));
	GpioLed.pGpioBase = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PUPD_PU;    // pull up but 40k Ohm, the current is too weak
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OTYPE_OPENDRAIN; // open drain

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GpioLed.pGpioBase, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();
	}
}

#elif defined(VIDEO_105)
void delay(void)
{
	uint32_t i;
	for (i=0; i<500000; ++i) { }
}

void video105_button_push_pull_led(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed, 0x00, sizeof(GpioLed));
	memset(&GpioBtn, 0x00, sizeof(GpioBtn));

	GpioLed.pGpioBase = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PUPD_NONE;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OTYPE_PUPL;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGpioBase = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PUPD_NONE;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
	//GpioBtn.GPIO_PinConfig.GPIO_PinOpType = GPIO_OTYPE_PUPL;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);


	while(1)
	{
		if (DISABLE == GPIO_ReadFromInputPin(GpioBtn.pGpioBase, GpioBtn.GPIO_PinConfig.GPIO_PinNumber))
		{
			GPIO_ToggleOutputPin(GpioLed.pGpioBase, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
			delay();
		}
	}
}

#elif defined(VIDEO_115)
void EXTI0_IRQHandler(void)
{
	// handle the EXTI0 interrupt
	GPIO_IrqHandling(0);
}

void delay(void)
{
	uint32_t i;
	for (i=0; i<500000; ++i) { }
}

void video115_push_button_interrupt(void)
{

}

#endif

int main(void)
{
#if defined(VIDEO_103)
	video103_push_pull_led();
#elif defined(VIDEO_104)
	video104_open_drain_led();
#elif defined(VIDEO_105)
	video105_button_push_pull_led();
#elif defined(VIDEO_115)
	video115_push_button_interrupt();
#endif
	return 0;
}
