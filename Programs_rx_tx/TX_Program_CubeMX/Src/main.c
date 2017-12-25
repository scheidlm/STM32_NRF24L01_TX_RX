/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/* 
  Programa oputue acp  kogni 45 mksec (po taimeru 3). Chastota duskretuzacii 
	vuxodut 22 kHz. Buffer zapovnuetsya 4erez DMA. Pislya zapovnennya polovunu buffera 
	(16 vidlikiv) generuetsya pereruvannya.  Funkcii vidpravlyaut dani 
	(16 vidlikiv) do NRF24L01.  
	 

		1910 -- sample rate 21980 Hz
		1908 -- 22000
		p.s. you can make filter if you want.
*/
//#include <stdio.h>
#include "defines.h"


#include "tm_stm32_hd44780.h"
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_nrf24l01_2mod.h"
//#include "tm_stm32_exti.h"
#include <string.h>
//#include "filter.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// polovuna buffera
#define TX_SIZE 16 
// kilkist elementviv buffera
#define SIZE 2*TX_SIZE
// kilkist baitiv dlya NRF24
#define PAYLOAD 32
#define FREQS 4
uint16_t ADC1ConvertedValues[SIZE]; //PingPongIN 1  L
uint16_t ADC2ConvertedValues[SIZE]; //PingPongIN 2  R
/* Buffer for strings */
char str[100];
uint8_t lowbit = 0;
uint8_t cnt_ch1 = 0;
uint8_t cnt_ch2 = 1;
uint8_t cnt_pwr = TM_NRF24L01_OutputPower_M18dBm;
///
#define PING 1
#define PONG 3

uint16_t  LpingOUT[TX_SIZE];  
uint16_t  LpongOUT[TX_SIZE];
uint16_t  RpingOUT[TX_SIZE];
uint16_t  RpongOUT[TX_SIZE];
uint16_t Lprocbuffer=0, Rprocbuffer=0;
volatile int16_t LRxcomplete2=0, LRxcomplete=0;
volatile int16_t RRxcomplete2=0, RRxcomplete=0;
volatile int16_t Txcomplete=1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
void Rprocess_buffer(void);
void Lprocess_buffer(void);
/* Private function prototypes -----------------------------------------------*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// data for transmite
/* My address */
uint8_t MyAddress[] = {
	0xE7,
	0xE7,
	0xE7,
	0xE7,
	0xE7
};
/* Receiver address */
uint8_t TxAddress[] = {
	0x7E,
	0x7E,
	0x7E,
	0x7E,
	0x7E
};



// ! you shoud make more distance between freq
uint8_t freq[FREQS] ={95, 100, 110, 120}; // 2400+ n
int8_t powers[FREQS] ={-18, -12, -6, 0};
//filter
//arm_fir_instance_q15 inst;
//#define NUM_TAPS N+1
//q15_t Data[NUM_TAPS+TX_SIZE];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	// filter
//	arm_fir_init_q15(&inst, NUM_TAPS, B1, Data, TX_SIZE);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	// Initialize HD44780
	TM_HD44780_Init(16, 2);
	/* Save custom character on location 0 in LCD */
	//	TM_HD44780_CreateChar(0, customChar);
	TM_HD44780_Clear();
	/* Put string to LCD */
	sprintf(str,"1: f%d %ddBm", cnt_ch1 +1, powers[cnt_pwr]); // 2400+ n
	//strcpy(str,"f =  MHz"); 
	TM_HD44780_Puts(0, 0, str);
	sprintf(str,"2: f%d %ddBm", cnt_ch2 +1, powers[cnt_pwr]); 
	//strcpy(str,"f =  MHz"); 
	TM_HD44780_Puts(0, 1, str);
	
	/* Initialize NRF24L01+ on channel 15 and 32bytes of payload */
	/* By default 2Mbps data rate and 0dBm output power */
	/* NRF24L01 goes to RX mode by default */
	TM_NRF24L01_Init(freq[cnt_ch1], PAYLOAD);
	/* Go to TX mode */
	TM_NRF24L01_PowerUpTx();
	/* Set 1MBps data rate and 0dBm output power */
	TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_1M, (TM_NRF24L01_OutputPower_t)cnt_pwr);
	/* Set my address, 5 bytes */
	TM_NRF24L01_SetMyAddress(MyAddress);
	/* Set TX address, 5 bytes */
	TM_NRF24L01_SetTxAddress(TxAddress);
// second NRF
	TM_NRF24L01_Init_2(freq[cnt_ch2], PAYLOAD);
	/* Go to TX mode */
	TM_NRF24L01_PowerUpTx_2();
	/* Set 1MBps data rate and 0dBm output power */
	TM_NRF24L01_SetRF_2(TM_NRF24L01_DataRate_1M, (TM_NRF24L01_OutputPower_t)cnt_pwr);
	/* Set my address, 5 bytes */
	TM_NRF24L01_SetMyAddress_2(MyAddress);
	/* Set TX address, 5 bytes */
	TM_NRF24L01_SetTxAddress_2(TxAddress);

	
	// -- Enables ADC DMA request //  ADC1 -> TIM3
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, SIZE) != HAL_OK) 
			return 0;  

		
			// -- Enables ADC DMA request //  ADC2 -> TIM2
		if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, SIZE) != HAL_OK) 
			return 0;  
		
				// -- Enables timer and timer will start conversion of the regular channels. 
	  if( HAL_TIM_Base_Start(&htim3) != HAL_OK) 
			return 0; 
				
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//		while((!LTxcomplete)|(!LRxcomplete));
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
//		Lprocess_buffer(); // and TX
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
//		while((!RTxcomplete)|(!RRxcomplete));
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
//		Rprocess_buffer(); // and TX
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
		while(!(LRxcomplete || LRxcomplete2));  //  1 0   
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);
		Lprocess_buffer(); // and TX
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
		while(!(RRxcomplete || RRxcomplete2));
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
		Rprocess_buffer(); // and TX
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1910;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1910;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  
{
	if (Txcomplete == 1){
    if (GPIO_Pin== BTN_PIN_PW) //x:0-15
    {
			HAL_TIM_Base_Stop(&htim3);
			HAL_ADC_Stop_DMA(&hadc1); 
			HAL_ADC_Stop_DMA(&hadc2);
			
	//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        	if( ++cnt_pwr == FREQS){
						cnt_pwr = 0;
					}
		
				TM_NRF24L01_ReSetChAndPow_2(freq[cnt_ch2], (TM_NRF24L01_OutputPower_t)cnt_pwr, TM_NRF24L01_DataRate_1M);
				TM_NRF24L01_ReSetChAndPow(freq[cnt_ch1], (TM_NRF24L01_OutputPower_t)cnt_pwr, TM_NRF24L01_DataRate_1M);

					TM_HD44780_Clear();
			/* Put string to LCD */
					sprintf(str,"1: f%d %ddBm", cnt_ch1 +1, powers[cnt_pwr]); 
				//strcpy(str,"f =  MHz"); 
				TM_HD44780_Puts(0, 0, str);
					sprintf(str,"2: f%d %ddBm", cnt_ch2 +1, powers[cnt_pwr]); 
				//strcpy(str,"f =  MHz"); 
				TM_HD44780_Puts(0, 1, str);
					
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, SIZE); 
					HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, SIZE);
					HAL_TIM_Base_Start(&htim3);
						
    }

    if (GPIO_Pin==BTN_PIN_CH) //y:0-15
    {
        HAL_TIM_Base_Stop(&htim3);
				HAL_ADC_Stop_DMA(&hadc1); 
				HAL_ADC_Stop_DMA(&hadc2);
			
			//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
        	if( ++cnt_ch1 == FREQS){
						cnt_ch1 = 0;
					}
					if( ++cnt_ch2 == FREQS){
						cnt_ch2 = 0;
					}
		
				TM_NRF24L01_ReSetChAndPow_2(freq[cnt_ch2],(TM_NRF24L01_OutputPower_t) cnt_pwr, TM_NRF24L01_DataRate_1M);
				TM_NRF24L01_ReSetChAndPow(freq[cnt_ch1], (TM_NRF24L01_OutputPower_t)cnt_pwr, TM_NRF24L01_DataRate_1M);

					TM_HD44780_Clear();
			/* Put string to LCD */
					sprintf(str,"1: f%d %ddBm", cnt_ch1 +1, powers[cnt_pwr]); 
				//strcpy(str,"f =  MHz"); 
				TM_HD44780_Puts(0, 0, str);
					sprintf(str,"2: f%d %ddBm", cnt_ch2 +1, powers[cnt_pwr]); 
				//strcpy(str,"f =  MHz"); 
				TM_HD44780_Puts(0, 1, str);
					
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, SIZE); 
					HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2ConvertedValues, SIZE);
					HAL_TIM_Base_Start(&htim3);
    }
	}
// check if data was TX 
//		if (GPIO_Pin==NRF24L01_IRQ_PIN) //y:0-15
//    {
//				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
//		}
}


// second part
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	if(hadc ==  &hadc1){		
		Lprocbuffer = PONG;
		LRxcomplete2 = 1;
	}
	
	if(hadc ==  &hadc2){
		Rprocbuffer = PONG;
		RRxcomplete2 = 1;
	}
}
// first part 	
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc ==  &hadc1){
		Lprocbuffer = PING;
		LRxcomplete = 1;
	}
	
	if(hadc ==  &hadc2){
		Rprocbuffer = PING;
		RRxcomplete = 1;
	}
	
}

void Lprocess_buffer(void)
{
	uint16_t *inBuf, *outBuf;
	uint16_t i;

	if (Lprocbuffer == PING){ 
		inBuf = ADC1ConvertedValues; 
		outBuf = LpingOUT; 
	}
	if (Lprocbuffer == PONG){
		inBuf = ADC1ConvertedValues+TX_SIZE; 
		outBuf = LpongOUT; 
	}
	// process
	for (i = 0; i < (TX_SIZE) ; i++){
		outBuf[i] = inBuf[i];
		inBuf[i]=0;
	}
	//arm_fir_q15(&inst, inBuf, outBuf, TX_SIZE);
	
	if (Lprocbuffer == PING){  
		LRxcomplete = 0;
	}else{
		LRxcomplete2 = 0;
	}
	Txcomplete = 0;
	// ready to TX
	TM_NRF24L01_Transmit((uint8_t*)(outBuf));
//	for (i = 0; i < (TX_SIZE) ; i++){
//		outBuf[i] = 0;
//	}
	Txcomplete = 1;
}

void Rprocess_buffer(void)
{
	uint16_t *inBuf, *outBuf;
	uint16_t i;

	if (Rprocbuffer == PING){ 
		inBuf = ADC2ConvertedValues; 
		outBuf = RpingOUT; 
	}
	if (Rprocbuffer == PONG){
		inBuf = ADC2ConvertedValues+TX_SIZE; 
		outBuf = RpongOUT; 
	}
	// process
	for (i = 0; i < (TX_SIZE) ; i++){
		outBuf[i] = inBuf[i];
		inBuf[i]=0;
	}
	
	if (Rprocbuffer == PING){  
		RRxcomplete = 0;
	}else{
		RRxcomplete2 = 0;
	}
	
	Txcomplete=0;
	// ready to TX
	TM_NRF24L01_Transmit_2((uint8_t*)(outBuf));
//	for (i = 0; i < (TX_SIZE) ; i++){
//		outBuf[i] = 0;
//	}
	Txcomplete = 1;
}


//void filter (uint16_t* TxData){
//				int32_t out;
//				int16_t *p;
//				int16_t * coeffs = B1;
//						*pSound = TxData[0];
//						out = 0;
//						p = pSound;
//						if(++pSound > &xSound[N]){
//							pSound = xSound;
//						}
//						for ( k=0; k<=N; ++k){
//							//out += (int32_t)(*p--) * (int32_t)(B[k]);
//							out = __SMLAD((*p--),coeffs[k], out);
//							if(p < &xSound[0]){
//								p = &xSound[N];
//								
//							}
//						}

//						out = (q15_t) (__SSAT((out >> 15), 16));
//						TxData[0] = (uint16_t)(out);
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
