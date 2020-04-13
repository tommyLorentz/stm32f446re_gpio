/*
 * stm32f446xx.h
 *
 *  Created on: Apr 12, 2020
 *      Author: tommy
 */


#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

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
#define APB1_PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR			0x40010000U
#define AHB1_PERIPH_BASEADDR			0x40020000U
#define AHB2_PERIPH_BASEADDR			0x50000000U
#define AHB3_PERIPH_BASEADDR			0x60000000U

#endif /* INC_STM32F446XX_H_ */
