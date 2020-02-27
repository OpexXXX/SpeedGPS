
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "GpsHadler.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;
osThreadId DynamicLedDriveHandle;
osThreadId UsartTask05Handle;
osThreadId gpsRXtaskHandle;
osThreadId GPSHadlerTaskHandle;
osThreadId DysplayTaskHandle;
osMessageQId DysplayQueue01Handle;
osMessageQId UsartTXQueue04Handle;
osMessageQId gpsSppedCoordsHandle;
osMessageQId GPSHandlerHandle;
osMessageQId DyspalyQueueAfterHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void StartDynamicLedDrive(void const * argument);
void UsartStartTask05(void const * argument);
void GpsTask(void const * argument);
void GPSHadlerFunc(void const * argument);
void DysplayTaskFunc(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t receiveBuffer[32];
Gps::GpsHadler gpsParser;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */



int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DynamicLedDrive */
  osThreadDef(DynamicLedDrive, StartDynamicLedDrive, osPriorityNormal, 0, 128);
  DynamicLedDriveHandle = osThreadCreate(osThread(DynamicLedDrive), NULL);

  /* definition and creation of UsartTask05 */
  osThreadDef(UsartTask05, UsartStartTask05, osPriorityLow, 0, 64);
  UsartTask05Handle = osThreadCreate(osThread(UsartTask05), NULL);

  /* definition and creation of gpsRXtask */
  osThreadDef(gpsRXtask, GpsTask, osPriorityNormal, 0, 64);
  gpsRXtaskHandle = osThreadCreate(osThread(gpsRXtask), NULL);

  /* definition and creation of GPSHadlerTask */
  osThreadDef(GPSHadlerTask, GPSHadlerFunc, osPriorityBelowNormal, 0, 192);
  GPSHadlerTaskHandle = osThreadCreate(osThread(GPSHadlerTask), NULL);

  /* definition and creation of DysplayTask */
  osThreadDef(DysplayTask, DysplayTaskFunc, osPriorityLow, 0, 128);
  DysplayTaskHandle = osThreadCreate(osThread(DysplayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of DysplayQueue01 */
/* what about the sizeof here??? cd native code */
  osMessageQDef(DysplayQueue01, 2, uint32_t);
  DysplayQueue01Handle = osMessageCreate(osMessageQ(DysplayQueue01), NULL);

  /* definition and creation of UsartTXQueue04 */
/* what about the sizeof here??? cd native code */
  osMessageQDef(UsartTXQueue04, 16, const char*);
  UsartTXQueue04Handle = osMessageCreate(osMessageQ(UsartTXQueue04), NULL);

  /* definition and creation of gpsSppedCoords */
/* what about the sizeof here??? cd native code */
  osMessageQDef(gpsSppedCoords, 16, uint8_t);
  gpsSppedCoordsHandle = osMessageCreate(osMessageQ(gpsSppedCoords), NULL);

  /* definition and creation of GPSHandler */
/* what about the sizeof here??? cd native code */
  osMessageQDef( GPSHandler, 10, Gps::gpsMessege);
   GPSHandlerHandle = osMessageCreate(osMessageQ( GPSHandler), NULL);

  /* definition and creation of DyspalyQueueAfter */
/* what about the sizeof here??? cd native code */
  osMessageQDef(DyspalyQueueAfter, 8, uint32_t);
  DyspalyQueueAfterHandle = osMessageCreate(osMessageQ(DyspalyQueueAfter), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  NVIC_EnableIRQ (USART2_IRQn); /*//разрешить прерывания от USART1*/

  USART2->CR1 |= USART_CR1_TCIE; /*//прерывание по окончанию передачи*/
  USART2->CR1 |= USART_CR1_RXNEIE; /*//прерывание по приему данных*/
  HAL_UART_Receive_IT (&huart2, receiveBuffer, (uint8_t) 1);



  while (1)
    {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
int iii=0;

    }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 500000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_UART_Receive_IT (&huart2, receiveBuffer, (uint8_t) 1);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  HAL_UART_Receive_IT (&huart2, receiveBuffer, (uint8_t) 1);
  /* Infinite loop */
  for (;;)
    {

      osDelay (2000);
    }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartDynamicLedDrive */
/**
* @brief Function implementing the DynamicLedDrive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDynamicLedDrive */
void StartDynamicLedDrive(void const * argument)
{
  /* USER CODE BEGIN StartDynamicLedDrive */

  /*//Обработка динамической индикации*/

  /* Infinite loop */
  for (;;)
    {
      osDelay (30000);
    }
  /* USER CODE END StartDynamicLedDrive */
}

/* USER CODE BEGIN Header_UsartStartTask05 */
/**
* @brief Function implementing the UsartTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UsartStartTask05 */
void UsartStartTask05(void const * argument)
{
  /* USER CODE BEGIN UsartStartTask05 */
  /* Infinite loop */
  for (;;)
    {
      osDelay (10000);
    }
  /* USER CODE END UsartStartTask05 */
}

/* USER CODE BEGIN Header_GpsTask */
/**
* @brief Function implementing the gpsRXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GpsTask */

void GpsTask(void const * argument)
{
  /* USER CODE BEGIN GpsTask */
  /*//ГПС парсер*/
  portBASE_TYPE xStatus;
  uint8_t gpsUartData;

  ssd1306_Init();
   HAL_Delay(1000);
   ssd1306_Fill(Black);
   ssd1306_UpdateScreen();

   HAL_Delay(1000);

   ssd1306_SetCursor(2,23);
   ssd1306_WriteString("Search GPS",Font_11x18,White);

   ssd1306_UpdateScreen();

  /* Infinite loop */
  for (;;)
    {
	  Gps::gpsMessege message;
      xStatus = xQueueReceive(gpsSppedCoordsHandle, &gpsUartData, 10);

      if (xStatus == pdPASS)
	{


    	 switch (gpsParser.charParser(gpsUartData)) {
			case Gps::GPS_NRMC:
			{

				message = gpsParser.getMessege();
				xStatus = xQueueSendToBack(GPSHandlerHandle, &message, 0);
				  ssd1306_Fill(Black);


				  char ress[20];
				  sprintf(ress, "%d", (message.Speed/1000));

				  uint8_t lenS = strlen(ress);
				  ssd1306_SetCursor(92-(lenS*16),10);
				  ssd1306_WriteString( ress,Font_16x26,White);
				  ssd1306_SetCursor(92,8);

				  sprintf(ress, "%d", (message.Speed%1000));

				 ssd1306_WriteChar(ress[0],Font_11x18,White);
				  ssd1306_SetCursor(92,24);
				  ssd1306_WriteString("km/ch",Font_7x10,White);

				  ssd1306_SetCursor(60,38);
				  ssd1306_WriteString("18.6",Font_11x18,White);
				  ssd1306_SetCursor(104,45);
				 ssd1306_WriteString("cek",Font_7x10,White);

				  ssd1306_SetCursor(5,0);
				  ssd1306_WriteString("2.2",Font_7x10,White);
				  ssd1306_SetCursor(5,11);
				  ssd1306_WriteString("4.6",Font_7x10,White);
				  ssd1306_SetCursor(5,22);
				  ssd1306_WriteString("6.7",Font_7x10,White);
				  ssd1306_SetCursor(5,33);
				  ssd1306_WriteString("10.3",Font_7x10,White);
				  ssd1306_SetCursor(5,44);



				 for (uint8_t var = 0; var < 128; ++var) {
					 ssd1306_DrawPixel(2,var,White);
				}
				 for (uint8_t var = 0; var < 128; ++var) {
									 ssd1306_DrawPixel(3,var,White);
								}
				   ssd1306_UpdateScreen();
			}

			break;
			default:
			break;
		}
	}
    }
  /* USER CODE END GpsTask */
}

/* USER CODE BEGIN Header_GPSHadlerFunc */
/**
* @brief Function implementing the GPSHadlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPSHadlerFunc */
void GPSHadlerFunc(void const * argument)
{
  /* USER CODE BEGIN GPSHadlerFunc */
  /* Infinite loop */
  portBASE_TYPE xStatus;
 Gps::gpsMessege gpsData;
  uint8_t counter = 0;

  for (;;)
    {





	      HAL_Delay(1000);


    }

  /* USER CODE END GPSHadlerFunc */
}

/* USER CODE BEGIN Header_DysplayTaskFunc */
/**
* @brief Function implementing the DysplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DysplayTaskFunc */
void DysplayTaskFunc(void const * argument)
{
  /* USER CODE BEGIN DysplayTaskFunc */


  /* Infinite loop */
  for (;;)
    {
	  HAL_Delay(1000);


    }
  /* USER CODE END DysplayTaskFunc */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
