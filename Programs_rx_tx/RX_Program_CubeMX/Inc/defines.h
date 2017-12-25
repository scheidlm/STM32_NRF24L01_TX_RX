/**
 * Defines for your entire project at one place
 * 
 * @author     Tilen Majerle
 * @email      tilen@majerle.eu
 * @website    http://stm32f4-discovery.net
 * @version    v1.0
 * @ide        Keil uVision 5
 * @license    GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2015
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

//#define STM32F051x8
#define ARM_MATH_CM0
#define STM32F0XX


/* Put your global defines for all libraries here used in your project */
//Change SPI used. Refer to TM SPI library to check which pins are for SPI
#define TM_SPI2_PRESCALER	  SPI_BAUDRATEPRESCALER_32
#define TM_SPI2_DATASIZE 	  SPI_DATASIZE_8BIT
#define TM_SPIx_MODE        TM_SPI_Mode_0

#define NRF24L01_SPI                SPI2
#define NRF24L01_SPI_PINS           TM_SPI_PinsPack_2

//Change CSN pin. This is for SPI communication
#define NRF24L01_CSN_PORT           GPIOA
#define NRF24L01_CSN_PIN            GPIO_Pin_9
//Change CE pin. This pin is used to enable/disable transmitter/receiver functionality
#define NRF24L01_CE_PORT            GPIOA
#define NRF24L01_CE_PIN             GPIO_Pin_10





/* Defines for RCC settings for system */
/* I've added these defines in options for target in Keil uVision for each target different settings */
//#define RCC_OSCILLATORTYPE    RCC_OSCILLATORTYPE_HSE /*!< Used to select system oscillator type */
//#define RCC_PLLM              8                      /*!< Used for PLL M parameter */
//#define RCC_PLLN              360                    /*!< Used for PLL N parameter */
//#define RCC_PLLP              2                      /*!< Used for PLL P parameter */
//#define RCC_PLLQ              7                      /*!< Used for PLL Q parameter */
//#define RCC_PLLR              10                     /*!< Used for PLL R parameter, available on STM32F446xx */

#endif
