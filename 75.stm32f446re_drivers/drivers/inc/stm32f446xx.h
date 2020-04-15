/*
 * stm32f446xx.h
 *
 *  Created on: Apr 12, 2020
 *      Author: tommy
 */


#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

/*
 * Generic macros
 */
#define __vo		volatile

#define ENABLE				1
#define DISABLE				0
#define SET					1
#define RESET				0
#define GPIO_SET			SET
#define GPIO_RESET			RESET

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
 * Note: register of a peripheral are specific to MCU.
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

typedef struct
{
	__vo uint32_t 	CR;				// 0x00, RCC clock control register
	__vo uint32_t 	PLLCFGR;		// 0x04, RCC PLL configuration register
	__vo uint32_t 	CFGR;			// 0x08, RCC clock configuration register
	__vo uint32_t 	CIR;			// 0x0C, RCC clock interrupt register
	__vo uint32_t 	AHB1RSTR;		// 0x10, RCC AHB1 peripheral reset register
	__vo uint32_t 	AHB2RSTR;		// 0x14, RCC AHB2 peripheral reset register
	__vo uint32_t 	AHB3RSTR;		// 0x18, RCC AHB3 peripheral reset register
	__vo uint32_t 	RESERVED0;		// 0x1C
	__vo uint32_t 	APB1RSTR;		// 0x20, RCC APB1 peripheral reset register
	__vo uint32_t 	APB2RSTR;		// 0x24, RCC APB2 peripheral reset register
	__vo uint32_t 	RESERVED1[2];	// 0x28, 0x2C
	__vo uint32_t 	AHB1ENR;		// 0x30, RCC AHB1 peripheral clock enable register
	__vo uint32_t 	AHB2ENR;		// 0x34, RCC AHB2 peripheral clock enable register
	__vo uint32_t 	AHB3ENR;		// 0x38, RCC AHB3 peripheral clock enable register
	__vo uint32_t 	RESERVED3;		// 0x3C
	__vo uint32_t 	APB1ENR;		// 0x40, RCC APB1 peripheral clock enable register
	__vo uint32_t 	APB2ENR;		// 0x44, RCC APB2 peripheral clock enable register
	__vo uint32_t 	RESERVED4[2];	// 0x48, 0x4C
	__vo uint32_t 	AHB1LPENR;		// 0x50, RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t 	AHB2LPENR;		// 0x54, RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t 	AHB3LPENR;		// 0x58, RCC AHB3 peripheral clock enable in low power mode register
	__vo uint32_t 	RESERVED5;		// 0x5C
	__vo uint32_t 	APB1LPENR;		// 0x60, RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t 	APB2LPENR;		// 0x64, RCC APB2 peripheral clock enable in low power mode register
	__vo uint32_t 	RESERVED6[2];	// 0x68, 0x6C
	__vo uint32_t 	BDCR;			// 0x70, RCC Backup domain control register
	__vo uint32_t 	CSR;			// 0x74, RCC clock control & status register
	__vo uint32_t 	RESERVED7[2];	// 0x78, 0x7C
	__vo uint32_t 	SSCGR;			// 0x80, RCC spread spectrum clock generation register
	__vo uint32_t 	PLLI2SCFGR;		// 0x84, RCC PLLI2S configuration register
	__vo uint32_t 	PLLSAICFGR;		// 0x88, RCC PLL configuration register
	__vo uint32_t 	DCKCFGR;		// 0x8C, RCC Dedicated Clock Configuration Register
	__vo uint32_t 	CKGATENR;		// 0x90, RCC clocks gated enable register
	__vo uint32_t 	DCKCFGR2;		// 0x94, RCC dedicated clocks configuration register 2
}RCC_RegDef_t;

/*
 * Note: register of a EXTI
 */
typedef struct
{
	__vo uint32_t 	IMR;		// 0x00, Interrupt mask register
	__vo uint32_t 	EMR;		// 0x04, Event mask register
	__vo uint32_t 	RTSR;		// 0x08, Rising trigger selection register
	__vo uint32_t 	FTSR;		// 0x0C, Falling trigger selection register
	__vo uint32_t 	SWIER;		// 0x10, Software interrupt event register
	__vo uint32_t 	PR;			// 0x14, Pending register

}EXTI_RegDef_t;

/*
 * Note: register of a SYSCFG
 */
typedef struct
{
	__vo uint32_t 	MEMRMP;			// 0x00, SYSCFG memory remap register
	__vo uint32_t 	PMC;			// 0x04, SYSCFG peripheral mode configuration register
	__vo uint32_t 	EXTICR[4];		// 0x08, 0x0C, 0x10, 0x14, SYSCFG external interrupt configuration register 1-4
	__vo uint32_t 	RESERVED1[2];	// 0x18, 0x1C
	__vo uint32_t 	CMPCR;			// 0x20, Compensation cell control register
	__vo uint32_t 	RESERVED2[2];	// 0x24, 0x28
	__vo uint32_t 	CFGR;			// 0x2C, SYSCFG configuration register

}SYSCFG_RegDef_t;


/*
 * Note: peripheral definition (Peripheral base addresses typecasted to xxx_RegDef_t
 */
#define GPIOA		((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t *) GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t *) RCC_BASEADDR)
#define EXTI		((EXTI_RegDef_t *) EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

/*
 * Return port code for GPIOx base address
 */
#define GPIO_BASEADDR_TO_PORTCODE(x)  ( (x == GPIOA)? 0 : \
										(x == GPIOB)? 1 : \
										(x == GPIOC)? 2 : \
										(x == GPIOD)? 3 : \
										(x == GPIOE)? 4 : \
										(x == GPIOF)? 5 : \
										(x == GPIOG)? 6 : 0 )


/*
 * Clock Enable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 0) )		// PCLK (peripheral clock)
#define GPIOB_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 1) )		// PCLK (peripheral clock)
#define GPIOC_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 2) )		// PCLK (peripheral clock)
#define GPIOD_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 3) )		// PCLK (peripheral clock)
#define GPIOE_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 4) )		// PCLK (peripheral clock)
#define GPIOF_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 5) )		// PCLK (peripheral clock)
#define GPIOG_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 6) )		// PCLK (peripheral clock)
#define GPIOH_PCLK_ENABLE()		(RCC->AHB1ENR |= (1 << 7) )		// PCLK (peripheral clock)

/*
 * Clock Enable macros for I2Cx peripheral
 */
#define I2C1_PCLK_ENABLE()			(RCC->APB1ENR |= (1 << 21) )	// PCLK (peripheral clock)
#define I2C2_PCLK_ENABLE()			(RCC->APB1ENR |= (1 << 22) )	// PCLK (peripheral clock)
#define I2C3_PCLK_ENABLE()			(RCC->APB1ENR |= (1 << 23) )	// PCLK (peripheral clock)

/*
 * Clock Enable macros for SPIx peripheral
 */
#define SPI1_PCLK_ENABLE()			(RCC->APB2ENR |= (1 << 12) )	// PCLK (peripheral clock)
#define SPI2LPEN_PCLK_ENABLE()		(RCC->APB1LPENR |= (1 << 14) )	// PCLK (peripheral clock)
#define SPI3LPEN_PCLK_ENABLE()		(RCC->APB1LPENR |= (1 << 15) )	// PCLK (peripheral clock)

/*
 * Clock Enable macros for USARTx peripheral
 */

/*
 * Clock Enable macros for SYSCFG peripheral
 */



/*
 * Clock Disable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0) )		// PCLK (peripheral clock)
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1) )		// PCLK (peripheral clock)
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2) )		// PCLK (peripheral clock)
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3) )		// PCLK (peripheral clock)
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4) )		// PCLK (peripheral clock)
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 5) )		// PCLK (peripheral clock)
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 6) )		// PCLK (peripheral clock)
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7) )		// PCLK (peripheral clock)

/*
 * Clock Disable macros for I2Cx peripheral
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21) )	// PCLK (peripheral clock)
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22) )	// PCLK (peripheral clock)
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23) )	// PCLK (peripheral clock)

/*
 * Clock Disable macros for SPIx peripheral
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12) )	// PCLK (peripheral clock)
#define SPI2LPEN_PCLK_DI()		(RCC->APB1LPENR &= ~(1 << 14) )	// PCLK (peripheral clock)
#define SPI3LPEN_PCLK_DI()		(RCC->APB1LPENR &= ~(1 << 15) )	// PCLK (peripheral clock)

/*
 * Clock Disable macros for USARTx peripheral
 */


/*
 * Clock Disable macros for SYSCFG peripheral
 */


/*
 * RESET macros for GPIOx peripheral
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0) ); (RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)	// gpio port reset
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1) ); (RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)	// gpio port reset
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2) ); (RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)	// gpio port reset
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3) ); (RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)	// gpio port reset
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4) ); (RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)	// gpio port reset
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5) ); (RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)	// gpio port reset
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6) ); (RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)	// gpio port reset
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7) ); (RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)	// gpio port reset

#endif /* INC_STM32F446XX_H_ */
