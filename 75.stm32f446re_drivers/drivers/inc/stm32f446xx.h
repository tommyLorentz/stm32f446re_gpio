/*
 * stm32f446xx.h
 *
 *  Created on: Apr 12, 2020
 *      Author: tommy
 */


#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo	volatile


/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U //112KB
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM_ADDR						SRAM1_BASEADDR


/*
 * AHB and APB bus Peripheral base addresses
 */
#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U
#define AHB3PERIPH_BASEADDR				0x60000000U


/*
 * Base addresses of Peripheral which are hanging on [AHB1] bus
 */
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00U)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800U)
/*
 * Base addresses of Peripheral which are hanging on [APB1] bus
 */
#define TIM2_BASEADDR					(APB1PERIPH_BASEADDR + 0x0000U)
#define TIM3_BASEADDR					(APB1PERIPH_BASEADDR + 0x0400U)
#define TIM4_BASEADDR					(APB1PERIPH_BASEADDR + 0x0800U)
#define TIM5_BASEADDR					(APB1PERIPH_BASEADDR + 0x0C00U)
#define TIM6_BASEADDR					(APB1PERIPH_BASEADDR + 0x1000U)
#define TIM7_BASEADDR					(APB1PERIPH_BASEADDR + 0x1400U)

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00U)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800U)
#define USAR4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00U)
#define USAR5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00U)


/*
 * Base addresses of Peripheral which are hanging on [APB2] bus
 */
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00U)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800U)

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000U)
#define SDMMC_BASEADDR					(APB2PERIPH_BASEADDR + 0x2C00U)
#define ADC_BASEADDR					(APB2PERIPH_BASEADDR + 0x2000U)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400U)
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000U)

#define TIM8_BASEADDR					(APB2PERIPH_BASEADDR + 0x0400U)
#define TIM1_BASEADDR					(APB2PERIPH_BASEADDR + 0x0000U)

/*
 * Note: register og a peripheral are specific to MCU.
 */
typedef struct
{
	__vo uint32_t 	MODER;		// 0x00, GPIO port mode register (i/o/alt/alg)
	__vo uint32_t 	OTYPER;		// 0x04, GPIO port output type register (push-pull/ open-drain)
	__vo uint32_t 	OSPEEDER;	// 0x08, GPIO port output speed register (Low/Medium,Fast/High)
	__vo uint32_t 	PUPDR;		// 0x0C, GPIO port pull-up/pull-down register(No PP/Pull up/Pull down/Reserved)
	__vo uint32_t 	IDR;		// 0x10, GPIO port input data register(read input data)
	__vo uint32_t 	ODR;		// 0x14, GPIO port output data register(R/W)
	__vo uint32_t 	BSRR;		// 0x18, GPIO port bit set/reset register(Word and Write only)
	__vo uint32_t 	LCKR;		// 0x1C, GPIO port configuration lock register
	__vo uint32_t 	AFR[2];		// 0x20, GPIO alternate function low(GP0-7), high(GP8-15) register(AFx selection)

}GPIO_RegDef_t;

#endif /* INC_STM32F446XX_H_ */
