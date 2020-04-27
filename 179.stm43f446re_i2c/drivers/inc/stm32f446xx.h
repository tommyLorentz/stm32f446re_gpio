/*
 * stm32f446xx.h
 *
 *  Created on: Apr 12, 2020
 *      Author: tommy
 */


#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>

/*
 * Generic macros
 */
#define __vo			volatile
#define __weak			__attribute__((weak))
#define __unused(x) 	((void) (x))

#define ENABLE				1
#define DISABLE				0
#define SET					1
#define RESET				0
#define GPIO_SET			SET
#define GPIO_RESET			RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET
#define ACK					SET
#define NACK				RESET

#define NVIC_GENERIC_OFFSET			0x4U

/* ARM cortex M4 processor NVIC ISERx register Address: Interrupt Set-enable Registers */
#define NVIC_ISER_BASE				0xE000E100U

/* ARM cortex M4 processor NVIC ISERx register Address: Interrupt Clear-enable Registers */
#define NVIC_ICER_BASE				0xE000E180U

/* ARM cortex M4 processor NVIC ISPRx register Address: Interrupt Set-pending Registers */
#define NVIC_ISPR_BASE				0xE000E200U

/* ARM cortex M4 processor NVIC ISERx register Address: Interrupt Clear-pending Registers */
#define NVIC_ICPR_BASE				0xE000E280U

/* ARM cortex M4 processor NVIC ISERx register Address: Interrupt Active Bit Registers */
#define NVIC_IABR_BASE				0xE000E300U

/* ARM cortex M4 processor NVIC ISERx register Address: Interrupt Priority Registers */
#define NVIC_PRI0					0xE000E400U

/* ARM cortex M4 processor NVIC ISERx register Address: Software Trigger Interrupt Register */
#define NVIC_STIR0					0xE000EF00U


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

#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400U)
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
 * Note: register of a SPI
 */
typedef struct
{
	__vo uint32_t 	CR1;			// 0x00, SPI control register 1
	__vo uint32_t 	CR2;			// 0x04, SPI control register 2
	__vo uint32_t 	SR;				// 0x08, SPI status register
	__vo uint32_t 	DR;				// 0x0C, SPI data register
	__vo uint32_t 	CRCPR;			// 0x10, SPI CRC polynomial register
	__vo uint32_t 	RXCRCR;			// 0x14, SPI RX CRC register
	__vo uint32_t 	TXCRCR;			// 0x18, SPI TX CRC register
	__vo uint32_t 	I2SCFGR;		// 0x1C, SPI_I2S configuration register
	__vo uint32_t 	I2SPR;			// 0x20, SPI_I2S prescaler register

}SPI_RegDef_t;

/*
 * Note: register of a I2C
 */
typedef struct
{
	__vo uint32_t 	CR1;			// 0x00, I2C Control register 1
	__vo uint32_t 	CR2;			// 0x04, I2C Control register 2
	__vo uint32_t 	OAR1;			// 0x08, I2C Own address register 1
	__vo uint32_t 	OAR2;			// 0x0C, I2C Own address register 2
	__vo uint32_t 	DR;				// 0x10, I2C Data register
	__vo uint32_t 	SR1;			// 0x14, I2C Status register 1
	__vo uint32_t 	SR2;			// 0x18, I2C Status register 2
	__vo uint32_t 	CCR;			// 0x1C, I2C Clock control register
	__vo uint32_t 	TRISE;			// 0x20, I2C TRISE register
	__vo uint32_t 	FLTR;			// 0x20, I2C FLTR register

}I2C_RegDef_t;

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

#define SPI1		((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t *) SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t *) I2C3_BASEADDR)

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
 * Fixed IRQ position of stm32f446xx mcu
 * Note: update these macros with valid values according to your mcu
 */
#define	IRQ_POSITION_EXIT0			6
#define	IRQ_POSITION_EXIT1			7
#define	IRQ_POSITION_EXIT2			8
#define	IRQ_POSITION_EXIT3			9
#define	IRQ_POSITION_EXIT4			10
#define	IRQ_POSITION_EXIT9_5		23
#define	IRQ_POSITION_SPI1			35
#define	IRQ_POSITION_SPI2			36
#define	IRQ_POSITION_EXIT15_10		40
#define	IRQ_POSITION_SPI3			51
#define	IRQ_POSITION_SPI4			84


/*
 * Maximum order of NVIC
 * system only use the higher 4 bits
 */
#define NVIC_PRI_MAX_ORDER			16
#define NVIC_PRI_BIT_SHIFT			4

/*
 * Settable IRQ priority of stm32f446xx mcu
 */
enum irq_priority_order {
	NVIC_PRI_EXTI0 = 0,
	NVIC_PRI_EXTI1,
	NVIC_PRI_EXTI2,
	NVIC_PRI_EXTI3,
	NVIC_PRI_EXTI4,
	NVIC_PRI_EXTI9_5 = 2,
	NVIC_PRI_EXTI15_10 = 3,
	NVIC_PRI_MAX = NVIC_PRI_MAX_ORDER
};


/**************************************************
 * Bit position of SPI peripheral
 *************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA_OFFSET			0	/* Clock phase */
#define SPI_CR1_CPOL_OFFSET			1	/* Clock polarity */
#define SPI_CR1_MSTR_OFFSET			2	/* Master selection */
#define SPI_CR1_BR_OFFSET			3	/* Baud rate control */
#define SPI_CR1_SPE_OFFSET			6	/* SPI enable */
#define SPI_CR1_LSBFIRST_OFFSET		7	/* Frame format */
#define SPI_CR1_SSI_OFFSET			8	/* Internal slave select */
#define SPI_CR1_SSM_OFFSET			9	/* Software slave management */
#define SPI_CR1_RXONLY_OFFSET		10	/* Receive only mode enable */
#define SPI_CR1_DFF_OFFSET			11	/* Data frame format */
#define SPI_CR1_CRCNEXT_OFFSET		12	/* CRC transfer next */
#define SPI_CR1_CRCEN_OFFSET		13	/* Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE_OFFSET		14	/* Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_OFFSET		15	/* Bidirectional data mode enable */

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_TXEIE_OFFSET		7	/* Tx buffer empty interrupt enable */
#define SPI_CR2_RXNEIE_OFFSET		6	/* RX buffer not empty interrupt enable */
#define SPI_CR2_ERRIE_OFFSET		5	/* Error interrupt enable */
#define SPI_CR2_FRF_OFFSET			4	/* Frame format */
#define SPI_CR2_SSOE_OFFSET			2	/* SS output enable */
#define SPI_CR2_TXDMAEN_OFFSET		1	/* Tx buffer DMA enable */
#define SPI_CR2_RXDMAEN_OFFSET		0	/* Rx buffer DMA enable */

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_FRE_OFFSET			8	/* Frame Error */
#define SPI_SR_BSY_OFFSET			7	/* Busy flag */
#define SPI_SR_OVR_OFFSET			6	/* Overrun flag */
#define SPI_SR_MODF_OFFSET			5	/* Mode fault */
#define SPI_SR_CRCERR_OFFSET		4	/* CRC error flag */
#define SPI_SR_UDR_OFFSET			3	/* Underrun flag */
#define SPI_SR_CHSIDE_OFFSET		2	/* Channel side */
#define SPI_SR_TXE_OFFSET			1	/* Transmit buffer empty */
#define SPI_SR_RXNE_OFFSET			0	/* Receive buffer not empty */

#define SPI_TXE_EMPTY		1
#define SPI_TXE_NEMPTY		0
#define SPI_RXNE_EMPTY		0
#define SPI_RXNE_NEMPTY		1

/**************************************************
 * Bit position of I2C peripheral
 *************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE_OFFSET			0	/* Peripheral enable */
#define I2C_CR1_SMBUS_OFFSET		1	/* SMBus mode */
#define I2C_CR1_SMBTYPE_OFFSET		3	/* SMBus type */
#define I2C_CR1_ENARP_OFFSET		4	/* ARP enable */
#define I2C_CR1_ENPEC_OFFSET		5	/* PEC enable */
#define I2C_CR1_ENGC_OFFSET			6	/* General call enable */
#define I2C_CR1_NOSTRETCH_OFFSET	7	/* Clock stretching disable (Slave mode) */
#define I2C_CR1_START_OFFSET		8	/* Start generation */
#define I2C_CR1_STOP_OFFSET			9	/* Stop generation */
#define I2C_CR1_ACK_OFFSET			10	/* Acknowledge enable */
#define I2C_CR1_POS_OFFSET			11	/* Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC_OFFSET			12	/* Packet error checking */
#define I2C_CR1_ALERT_OFFSET		14	/* SMBus alert */
#define I2C_CR1_SWRST_OFFSET		15	/* Software reset */

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_LAST_OFFSET			12	/* DMA last transfer */
#define I2C_CR2_DMAEN_OFFSET		11	/* DMA requests enable */
#define I2C_CR2_ITBUFEN_OFFSET		10	/* Buffer interrupt enable */
#define I2C_CR2_ITEVTEN_OFFSET		9	/* Event interrupt enable */
#define I2C_CR2_ITERREN_OFFSET		8	/* Error interrupt enable */
#define I2C_CR2_FREQ_OFFSET			0	/* Peripheral clock frequency */

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SMBALERT_OFFSET		15	/* SMBus alert */
#define I2C_SR1_TIMEOUT_OFFSET		14	/* Timeout or Tlow error  */
#define I2C_SR1_PECERR_OFFSET		12	/* PEC Error in reception */
#define I2C_SR1_OVR_OFFSET			11	/* Overrun/Underrun  */
#define I2C_SR1_AF_OFFSET			10	/* Acknowledge failure */
#define I2C_SR1_ARLO_OFFSET			9	/* Arbitration lost (master mode) */
#define I2C_SR1_BERR_OFFSET			8	/* Bus error */
#define I2C_SR1_TXE_OFFSET			7	/* Data register empty (transmitters) */
#define I2C_SR1_RxNE_OFFSET			6	/* Data register not empty (receivers) */
#define I2C_SR1_STOPF_OFFSET		4	/* Stop detection (slave mode) */
#define I2C_SR1_ADD10_OFFSET		3	/* 10-bit header sent (Master mode) */
#define I2C_SR1_BTF_OFFSET			2	/* Byte transfer finished */
#define I2C_SR1_ADDR_OFFSET			1	/* Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_SB_OFFSET			0	/* Start bit (Master mode) */

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_PEC_OFFSET			8	/* PEC[7:0] Packet error checking register */
#define I2C_SR2_DUALF_OFFSET		7	/* Dual flag (Slave mode) */
#define I2C_SR2_SMBHOST_OFFSET		6	/* SMBus host header (Slave mode) */
#define I2C_SR2_SMBDEFAULT_OFFSET	5	/* SMBus device default address (Slave mode) */
#define I2C_SR2_GENCALL_OFFSET		4	/* General call address (Slave mode) */
#define I2C_SR2_TRA_OFFSET			2	/* Transmitter/receiver */
#define I2C_SR2_BUSY_OFFSET			1	/* Bus busy */
#define I2C_SR2_MSL_OFFSET			8	/* Master/slave */

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_FS_DUTY_OFFSET			15	/* I2C master mode selection */
#define I2C_CCR_DUTY_OFFSET			14	/* Fm mode duty cycle */
#define I2C_CCR_CCR_OFFSET			0	/* CCR[11:0]: Clock control register in Fm/Sm mode (Master mode)  */


/*
 * Clock Enable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0) )		// PCLK (peripheral clock)
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1) )		// PCLK (peripheral clock)
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2) )		// PCLK (peripheral clock)
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3) )		// PCLK (peripheral clock)
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4) )		// PCLK (peripheral clock)
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5) )		// PCLK (peripheral clock)
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6) )		// PCLK (peripheral clock)
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7) )		// PCLK (peripheral clock)

/*
 * Clock Enable macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21) )	// PCLK (peripheral clock)
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22) )	// PCLK (peripheral clock)
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23) )	// PCLK (peripheral clock)

/*
 * Clock Enable macros for SPIx peripheral
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12) )	// PCLK (peripheral clock)
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14) )	// PCLK (peripheral clock)
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15) )	// PCLK (peripheral clock)
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13) )	// PCLK (peripheral clock)

/*
 * Clock Enable macros for USARTx peripheral
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4) )		// PCLK (peripheral clock)

/*
 * Clock Enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14) )	// PCLK (peripheral clock)

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
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14) )	// PCLK (peripheral clock)
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15) )	// PCLK (peripheral clock)
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13) )	// PCLK (peripheral clock)

/*
 * Clock Disable macros for USARTx peripheral
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4) )		// PCLK (peripheral clock)

/*
 * Clock Disable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14) )		// PCLK (peripheral clock)

/*
 * RESET macros for GPIOx peripheral
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0) ); (RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1) ); (RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2) ); (RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3) ); (RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4) ); (RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5) ); (RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6) ); (RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7) ); (RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)

/*
 * RESET macros for SPIx peripheral
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12) ); (RCC->APB2RSTR &= ~(1 << 12) ); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14) ); (RCC->APB1RSTR &= ~(1 << 14) ); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15) ); (RCC->APB1RSTR &= ~(1 << 15) ); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 13) ); (RCC->APB2RSTR &= ~(1 << 13) ); }while(0)


#endif /* INC_STM32F446XX_H_ */
