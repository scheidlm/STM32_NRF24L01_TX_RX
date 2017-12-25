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

#define STM32F4xx
//#define ARM_MATH_CM4 

/* Put your global defines for all libraries here used in your project */
#define DMA2_STREAM0_DISABLE_IRQHANDLER
#define DMA2_STREAM2_DISABLE_IRQHANDLER
#define TM_SPI1_PRESCALER   SPI_BAUDRATEPRESCALER_16
#define TM_SPI3_PRESCALER   SPI_BAUDRATEPRESCALER_8

/* Defines for RCC settings for system */
/* I've added these defines in options for target in Keil uVision for each target different settings */
//#define RCC_OSCILLATORTYPE    RCC_OSCILLATORTYPE_HSE /*!< Used to select system oscillator type */
//#define RCC_PLLM              4                      /*!< Used for PLL M parameter */
//#define RCC_PLLN              168                   /*!< Used for PLL N parameter */
//#define RCC_PLLP              2                      /*!< Used for PLL P parameter */
//#define RCC_PLLQ              4                      /*!< Used for PLL Q parameter */
//#define RCC_PLLR              10                     /*!< Used for PLL R parameter, available on STM32F446xx */

//Change SPI used. Refer to TM SPI library to check which pins are for SPI
#define NRF24L01_SPI				SPI1
#define NRF24L01_SPI_PINS			TM_SPI_PinsPack_1

//Change CSN pin. This is for SPI communication
#define NRF24L01_CSN_PORT			GPIOC
#define NRF24L01_CSN_PIN			GPIO_Pin_6

//Change CE pin. This pin is used to enable/disable transmitter/receiver functionality
#define NRF24L01_CE_PORT			GPIOC
#define NRF24L01_CE_PIN				GPIO_Pin_8
#define NRF24L01_IRQ_PIN			GPIO_Pin_7
/// second NRF

#define NRF24L01_SPI_2				SPI3
#define NRF24L01_SPI_2_PINS			TM_SPI_PinsPack_2

//Change CSN pin. This is for SPI communication
#define NRF24L01_CSN_PORT_2			GPIOA
#define NRF24L01_CSN_PIN_2			GPIO_Pin_11
#define NRF24L01_IRQ_PIN_2			GPIO_Pin_9

//Change CE pin. This pin is used to enable/disable transmitter/receiver functionality
#define NRF24L01_CE_PORT_2			GPIOA
#define NRF24L01_CE_PIN_2				GPIO_Pin_10


//////////////

//HD44780 LCD driver
//RS - Register select pin
#define HD44780_RS_PORT     GPIOB
#define HD44780_RS_PIN      GPIO_PIN_10
//E - Enable pin
#define HD44780_E_PORT      GPIOB
#define HD44780_E_PIN       GPIO_PIN_11
//D4 - Data 4 pin
#define HD44780_D4_PORT     GPIOB
#define HD44780_D4_PIN      GPIO_PIN_12
//D5 - Data 5 pin
#define HD44780_D5_PORT     GPIOB
#define HD44780_D5_PIN      GPIO_PIN_13
//D6 - Data 6 pin
#define HD44780_D6_PORT     GPIOB
#define HD44780_D6_PIN      GPIO_PIN_14
//D7 - Data 7 pin
#define HD44780_D7_PORT     GPIOB
#define HD44780_D7_PIN      GPIO_PIN_15


#define BTN_PORT        GPIOB
#define BTN_PIN_CH      GPIO_PIN_8
#define BTN_PIN_PW      GPIO_PIN_9

#endif
