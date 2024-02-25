/*
 * stm32f407xx.h
 *
 *  Created on: Feb 25, 2024
 *      Author: Nguyen Lam Anh Vu
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR				0x08000000U 		/*Base address of Flash*/
#define SRAM1_BASEADDR				0x20000000U			/*Base address of SRAM1*/
#define SRAM2_BASEADDR				0x2001C000U			/*Base address of SRAM2*/
#define ROM_BASEADDR				0x1FFF0000U			/*Base address of System memory*/
#define SRAM 						SRAM1_BASEADDR

/*
 *	AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR				0x40000000U			/*Base address of Peripheral*/
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR 	/*Base address of APB1*/
#define APB2PERIPH_BASEADDR			0x40010000U			/*Base address of APB2*/
#define AHB1PERIPH_BASEADDR			0x40020000U 		/*Base address of AHB1*/
#define AHB2PERIPH_BASEADDR			0x50000000U			/*Base address of AHB2*/

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				(APB1PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR				(APB1PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB1PERIPH_BASEADDR + 0x1400)

/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;				/*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;				/*!< GPIO port output type register,     			Address offset: 0x04      */
	__vo uint32_t OSPEEDR;				/*!< GPIO port output speed register,     			Address offset: 0x08      */
	__vo uint32_t PUPDR;				/*!< GPIO port pull-up/pull-down register,     		Address offset: 0x0C      */
	__vo uint32_t IDR;					/*!< GPIO port input data register,    				Address offset: 0x10      */
	__vo uint32_t ODR;					/*!< GPIO port output data register,     			Address offset: 0x14      */
	__vo uint32_t BSRR;					/*!< GPIO port bit set/reset register,     			Address offset: 0x18      */
	__vo uint32_t LCKR;					/*!< GPIO port configuration lock register,     	Address offset: 0x1C      */
	__vo uint32_t AFR[2];				/*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
	__vo uint32_t CR;			/*!< RCC clock control register,     				Address offset: 0x00 */
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register,     			Address offset: 0x04 */
	__vo uint32_t CFGR;			/*!< RCC clock configuration register,     			Address offset: 0x08 */
	__vo uint32_t CIR;			/*!< RCC clock interrupt register,     				Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register,     		Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register,     		Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register,   			Address offset: 0x18 */
	uint32_t 	  RESERVED0;	/*!< Reserved, 0x1C                                                       */
	__vo uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register,     		Address offset: 0x20 */
	__vo uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register,     		Address offset: 0x24 */
	uint32_t 	  RESERVED1[2];	/*!< Reserved, 0x28-0x2C                                                  */
	__vo uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock register,     		Address offset: 0x30 */
	__vo uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register,     Address offset: 0x34 */
	__vo uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register,    	Address offset: 0x38 */
	uint32_t 	  RESERVED2;	/*!< Reserved, 0x3C                                                       */
	__vo uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register,    	Address offset: 0x40 */
	__vo uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register,  	Address offset: 0x44 */
	uint32_t 	  RESERVED3[2];	/*!< Reserved, 0x48-0x4C                                                  */
	__vo uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register,  		Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register,     	Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register,		Address offset: 0x58 */
	uint32_t 	  RESERVED4;	/*!< Reserved, 0x5C                                                       */
	__vo uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register,   	Address offset: 0x60 */
	__vo uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enabled in low power mode register, 		Address offset: 0x64 */
	uint32_t 	  RESERVED5[2];	/*!< Reserved, 0x68-0x6C                                                  */
	__vo uint32_t BDCR;			/*!< RCC Backup domain control register,     		Address offset: 0x70 */
	__vo uint32_t CSR;			/*!< RCC clock control & status register,     		Address offset: 0x74 */
	uint32_t 	  RESERVED6[2];	/*!< Reserved, 0x78-0x7C                                                  */
	__vo uint32_t SSCGR;		/*!< RCC clock control & status register,    		Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register,     		Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR;   /*!< RCC PLL configuration register,     			Address offset: 0x88 */
	__vo uint32_t DCKCFGR;      /*!< RCC Dedicated Clock Configuration Register,    Address offset: 0x8C */
	__vo uint32_t CKGATENR;     /*!< TODO,     										Address offset: 0x90 */
	__vo uint32_t DCKCFGR2;     /*!< TODO,     										Address offset: 0x94 */
} RCC_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)
#endif /* INC_STM32F407XX_H_ */
