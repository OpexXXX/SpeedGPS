
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
#include "ledDriver.h"
#include  "keyboardDriver.h"
#include  "buzzerDriver.h"
#include  "gps.h"
#include "ParameterMeter.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;
osThreadId DynamicLedDriveHandle;
osThreadId KeybordDriverHandle;
osThreadId BuzzerTaskHandle;
osThreadId UsartTask05Handle;
osThreadId gpsRXtaskHandle;
osThreadId GPSHadlerTaskHandle;
osThreadId DysplayTaskHandle;
osMessageQId DysplayQueue01Handle;
osMessageQId PressedKeyQueue02Handle;
osMessageQId BuzzerQueue03Handle;
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
void StartDefaultTask(void const * argument);
void StartDynamicLedDrive(void const * argument);
void StartKeybordDriver(void const * argument);
void StartBuzzerTask04(void const * argument);
void UsartStartTask05(void const * argument);
void GpsTask(void const * argument);
void GPSHadlerFunc(void const * argument);
void DysplayTaskFunc(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t receiveBuffer[32];
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
	osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of DynamicLedDrive */
	osThreadDef(DynamicLedDrive, StartDynamicLedDrive, osPriorityLow, 0, 128);
	DynamicLedDriveHandle = osThreadCreate(osThread(DynamicLedDrive), NULL);

	/* definition and creation of KeybordDriver */
	osThreadDef(KeybordDriver, StartKeybordDriver, osPriorityLow, 0, 128);
	KeybordDriverHandle = osThreadCreate(osThread(KeybordDriver), NULL);

	/* definition and creation of BuzzerTask */
	osThreadDef(BuzzerTask, StartBuzzerTask04, osPriorityLow, 0, 128);
	BuzzerTaskHandle = osThreadCreate(osThread(BuzzerTask), NULL);

	/* definition and creation of UsartTask05 */
	osThreadDef(UsartTask05, UsartStartTask05, osPriorityIdle, 0, 128);
	UsartTask05Handle = osThreadCreate(osThread(UsartTask05), NULL);

	/* definition and creation of gpsRXtask */
	osThreadDef(gpsRXtask, GpsTask, osPriorityNormal, 0, 128);
	gpsRXtaskHandle = osThreadCreate(osThread(gpsRXtask), NULL);

	/* definition and creation of GPSHadlerTask */
	osThreadDef(GPSHadlerTask, GPSHadlerFunc, osPriorityBelowNormal, 0, 128);
	GPSHadlerTaskHandle = osThreadCreate(osThread(GPSHadlerTask), NULL);

	/* definition and creation of DysplayTask */
	osThreadDef(DysplayTask, DysplayTaskFunc, osPriorityLow, 0, 128);
	DysplayTaskHandle = osThreadCreate(osThread(DysplayTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of DysplayQueue01 */
	osMessageQDef(DysplayQueue01, 2, dysplayBufferStruct);
	DysplayQueue01Handle = osMessageCreate(osMessageQ(DysplayQueue01), NULL);

	/* definition and creation of PressedKeyQueue02 */
	osMessageQDef(PressedKeyQueue02, 5, pressedKeyStruct);
	PressedKeyQueue02Handle = osMessageCreate(osMessageQ(PressedKeyQueue02), NULL);

	/* definition and creation of BuzzerQueue03 */
	osMessageQDef(BuzzerQueue03, 5, buzzerStruct);
	BuzzerQueue03Handle = osMessageCreate(osMessageQ(BuzzerQueue03), NULL);

	/* definition and creation of UsartTXQueue04 */
	osMessageQDef(UsartTXQueue04, 16, const char*);
	UsartTXQueue04Handle = osMessageCreate(osMessageQ(UsartTXQueue04), NULL);

	/* definition and creation of gpsSppedCoords */
	osMessageQDef(gpsSppedCoords, 16, uint8_t);
	gpsSppedCoordsHandle = osMessageCreate(osMessageQ(gpsSppedCoords), NULL);

	/* definition and creation of GPSHandler */
	osMessageQDef( GPSHandler, 10, gpsSpeedMessegeStruct);
	GPSHandlerHandle = osMessageCreate(osMessageQ( GPSHandler), NULL);

	/* definition and creation of DyspalyQueueAfter */
	osMessageQDef(DyspalyQueueAfter, 8, dysplayBufferStruct);
	DyspalyQueueAfterHandle = osMessageCreate(osMessageQ(DyspalyQueueAfter), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */


	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	NVIC_EnableIRQ (USART2_IRQn);           //разрешить прерывания от USART1



	USART2->CR1  |= USART_CR1_TCIE;         //прерывание по окончанию передачи
	USART2->CR1  |= USART_CR1_RXNEIE;       //прерывание по приему данных
	HAL_UART_Receive_IT(&huart2, receiveBuffer, (uint8_t)1);


	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

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
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

	/* USER CODE BEGIN 5 */
	portBASE_TYPE xStatus;
	dysplayBufferStruct  SymbolBuffer;
	uint16_t i = 0;
	HAL_UART_Receive_IT(&huart2, receiveBuffer, (uint8_t)1);
	/* Infinite loop */
	for(;;)
	{





		//	getLedBufferFromNumber(i, &SymbolBuffer);
		//xStatus = xQueueSendToBack(DysplayQueue01Handle,&SymbolBuffer,5);
		//	i++;
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

/* StartDynamicLedDrive function */
void StartDynamicLedDrive(void const * argument)
{
	/* USER CODE BEGIN StartDynamicLedDrive */

	//Обработка динамической индикации
	dysplayBufferStruct SymbolBuffer;
	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(DysplayQueue01Handle, &SymbolBuffer, 0);
		uint8_t aTxBuffer[2];
		aTxBuffer[1] = SymbolBuffer.firstReg;
		aTxBuffer[0] = SymbolBuffer.LedState;


		aTxBuffer[0] &= ~0b00001111;
		aTxBuffer[0] |= 0b00001000;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) aTxBuffer, 2, 5);
		cs_strob()
		;
		osDelay(3);
		aTxBuffer[1] = SymbolBuffer.secondReg;
		aTxBuffer[0] &= ~0b00001111;
		aTxBuffer[0] |= 0b00000100;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) aTxBuffer, 2, 5);
		cs_strob()
		;
		osDelay(3);
		aTxBuffer[1] = SymbolBuffer.thirdReg;
		aTxBuffer[0] &= ~0b00001111;
		aTxBuffer[0] |= 0b00000010;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) aTxBuffer, 2, 5);
		cs_strob()
		;
		osDelay(3);
		aTxBuffer[1] = SymbolBuffer.fourthReg;
		aTxBuffer[0] &= ~0b00001111;
		aTxBuffer[0] |= 0b00000001;
		HAL_SPI_Transmit(&hspi1, (uint8_t*) aTxBuffer, 2, 5);
		cs_strob()
		;
		osDelay(3);

	}
	/* USER CODE END StartDynamicLedDrive */
}

/* StartKeybordDriver function */
void StartKeybordDriver(void const * argument)
{
	/* USER CODE BEGIN StartKeybordDriver */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartKeybordDriver */
}

/* StartBuzzerTask04 function */
void StartBuzzerTask04(void const * argument)
{
	/* USER CODE BEGIN StartBuzzerTask04 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartBuzzerTask04 */
}

/* UsartStartTask05 function */
void UsartStartTask05(void const * argument)
{
	/* USER CODE BEGIN UsartStartTask05 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END UsartStartTask05 */
}

/* GpsTask function */
void GpsTask(void const * argument)
{
	/* USER CODE BEGIN GpsTask */
	//ГПС парсер
	portBASE_TYPE xStatus;
	uint8_t gpsUartData;
	/* Infinite loop */
	for(;;)
	{
		xStatus=xQueueReceive(gpsSppedCoordsHandle, &gpsUartData, 1);
		if (xStatus == pdPASS){
			uartParserGps(gpsUartData);
		}
	}
	/* USER CODE END GpsTask */
}

/* GPSHadlerFunc function */
void GPSHadlerFunc(void const * argument)
{
	/* USER CODE BEGIN GPSHadlerFunc */
	// Главный обработчик замеров
	/* Infinite loop */
	uint8_t VehicleStatus=0;
	uint8_t MeasurmentStatus=0;
	portBASE_TYPE xStatus;
	gpsSpeedMessegeStruct messageArray[5];
	gpsSpeedMessegeStruct gpsData;
	gpsSpeedMessegeStruct StartMeas;
	gpsSpeedInderStruct IntermediateRresults[30];

	for(;;)
	{
		dysplayBufferStruct SymBuffer;
		xStatus=xQueueReceive(GPSHandlerHandle, &gpsData, 1);
		if (xStatus == pdPASS ){
			//Проверочный коммит
			//Сдвигаем буфер с замерами
			for (uint8_t var = 0;  var < 4; ++var) {
				messageArray[var]=messageArray[var+1];
			}
			// Записываем текущий замер
			messageArray[4]=gpsData;

			if(gpsData.CourseTrue == 0) //Если курса нет
			{
				SymBuffer.LedState &= ~0b00010000; //Поджигаем зеленый идикатор
				getLedBufferFromNumberSpeed(0,&SymBuffer);
				SymBuffer.ShowDelay = 0;
				xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,0);}
			else   //Если курс есть
			{
				SymBuffer.LedState|= 0b00010000; //Тушим зеленый индикатор
				uint32_t AvgSpeed = (messageArray[0].Speed+ messageArray[1].Speed+ messageArray[2].Speed+ messageArray[3].Speed+ messageArray[4].Speed)/5; // Берем среднюю скорость пяти замеров
				getLedBufferFromNumberSpeed(AvgSpeed,&SymBuffer);
				SymBuffer.ShowDelay = 0;
				xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,0); // отображаем на экране без задержки
			}

			//Проверка на Остановку автомобиля, обнуление замеров, если 5 замеров отсутствует курс и скорость ниже 1 км/ч
			uint8_t flagRes =0;
			//прогоняем все пять замеров
			for (int var = 0; var < 5; ++var) {
				flagRes+=(messageArray[var].CourseTrue == 0); //проверяем отсутствие курса
				flagRes+=(messageArray[var].Speed <1000);// проверяем скорость ниже 1 км/ч
			}

			if(flagRes==10){VehicleStatus=VEHICLE_STOPPED;} //Если все пять замеров нулевые, считаем , что автомобиль остановлен и готов для старта

			if(VehicleStatus==VEHICLE_STOPPED){ // если автомобиль остановлен и готов для старта
				//обнулить промежуточные итоги
				SymBuffer.LedState &= ~0b10000000; // поджигаем красный индикатор
				MeasurmentStatus = MES_STOPPED; // статус измерения "остановлен"
			}
			//Проверка на начала замера с места
				//Если в текущем пакете ГПС появился курс, а в предыдущем он отсутствовал  и средняя скорость предыдущих двух замеров меньше 1 км/ч
			if(gpsData.CourseTrue!=0 && (messageArray[3].CourseTrue==0) && (((messageArray[3].Speed+messageArray[2].Speed)/2)<1000))
			{
				StartMeas =messageArray[3]; // Записываем точку старта
				VehicleStatus = VEHICLE_ACCELERATE; // статус автомобиля "Ускоряется"
				SymBuffer.LedState|= 0b10000000; // тушим красный диод
				MeasurmentStatus = MES_ACCELERATE;  // статус измерения "Ускоряется"
			}
			//Если находимся в режиме замера ускорения
			if(MeasurmentStatus == MES_ACCELERATE)
			{
				/*//при переходе порога 5 км/ч
				if(gpsData.Speed>5000 &&  messageArray[3].Speed<5000)
				{
					uint32_t resultTime = getDifTime(StartMeas.Time, messageArray[3].Time);
					float gpsDataSpeedF=gpsData.Speed;
					float messageArraySpeedF=messageArray[3].Speed;
					float koef = 1- ((gpsDataSpeedF-5000)/(gpsDataSpeedF-messageArraySpeedF));
					uint32_t difTime =( getDifTime(messageArray[3].Time, gpsData.Time))*koef;
					resultTime+=difTime;

					getLedBufferFromNumberTime(resultTime,&SymBuffer);
					SymBuffer.ShowDelay = 500;
					xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,5);

					//отобразить результат 5 кмч
				}*/

				if(gpsData.Speed>30000 &&  messageArray[3].Speed<30000)
				{

					uint32_t resultTime = getDifTime(StartMeas.Time, messageArray[3].Time);
					float gpsDataSpeedF=gpsData.Speed;
					float messageArraySpeedF=messageArray[3].Speed;
					float koef = 1- ((gpsDataSpeedF-30000)/(gpsDataSpeedF-messageArraySpeedF));
					uint32_t difTime =( getDifTime(messageArray[3].Time, gpsData.Time))*koef;
					resultTime+=difTime;
					getLedBufferFromNumberTime(resultTime,&SymBuffer);
					SymBuffer.ShowDelay = 1000;
					xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,5);

					//отобразить результат 30 кмч

				}
				if(gpsData.Speed>60000 &&  messageArray[3].Speed<60000)
				{
					uint32_t resultTime = getDifTime(StartMeas.Time, messageArray[3].Time);
					float gpsDataSpeedF=gpsData.Speed;
					float messageArraySpeedF=messageArray[3].Speed;
					float koef = 1- ((gpsDataSpeedF-60000)/(gpsDataSpeedF-messageArraySpeedF));
					uint32_t difTime =( getDifTime(messageArray[3].Time, gpsData.Time))*koef;
					resultTime+=difTime;
					getLedBufferFromNumberTime(resultTime,&SymBuffer);
					SymBuffer.ShowDelay = 2300;
					xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,5);

					//отобразить результат 60 кмч

				}
				if(gpsData.Speed>100000 &&  messageArray[3].Speed<100000)
				{
					uint32_t resultTime = getDifTime(StartMeas.Time, messageArray[3].Time);
					float gpsDataSpeedF=gpsData.Speed;
					float messageArraySpeedF=messageArray[3].Speed;
					float koef = 1- ((gpsDataSpeedF-100000)/(gpsDataSpeedF-messageArraySpeedF));
					uint32_t difTime =( getDifTime(messageArray[3].Time, gpsData.Time))*koef;
					resultTime+=difTime;
					getLedBufferFromNumberTime(resultTime,&SymBuffer);
					//StartMeas.Time = StartMeas.Time+resultTime;
					SymBuffer.ShowDelay = 4000;
					xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,5);

					//отобразить результат  100кмч
				}
				if(gpsData.Speed>150000 &&  messageArray[3].Speed<150000)
				{
					uint32_t resultTime = getDifTime(StartMeas.Time, messageArray[3].Time);
					float gpsDataSpeedF=gpsData.Speed;
					float messageArraySpeedF=messageArray[3].Speed;
					float koef = 1- ((gpsDataSpeedF-150000)/(gpsDataSpeedF-messageArraySpeedF));
					uint32_t difTime =( getDifTime(messageArray[3].Time, gpsData.Time))*koef;
					resultTime+=difTime;
					getLedBufferFromNumberTime(resultTime,&SymBuffer);
					SymBuffer.ShowDelay = 5000;
					xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,5);
					//отобразить результат 150 кмч
				}
				if(gpsData.Speed>200000 &&  messageArray[3].Speed<200000)
				{
					uint32_t resultTime = getDifTime(StartMeas.Time, messageArray[3].Time);
					float gpsDataSpeedF=gpsData.Speed;
					float messageArraySpeedF=messageArray[3].Speed;
					float koef = 1- ((gpsDataSpeedF-200000)/(gpsDataSpeedF-messageArraySpeedF));
					uint32_t difTime =( getDifTime(messageArray[3].Time, gpsData.Time))*koef;
					resultTime+=difTime;
					getLedBufferFromNumberTime(resultTime,&SymBuffer);
					SymBuffer.ShowDelay = 9000;
					xQueueSendToBack(DyspalyQueueAfterHandle,&SymBuffer,5);
					//отобразить результат 200 кмч
				}
				/*for (int var = 0; var < 30; ++var)
				{
					//Если перешагнули порог указанный в IntermediateRresults
					if(gpsData.Speed>IntermediateMeasurementOfSpeed[var] &&  messageArray[3].Speed<IntermediateMeasurementOfSpeed[var])
					{
						//Копируем Результат
						IntermediateRresults[var].Speed = gpsData.Speed;
						IntermediateRresults[var].Time = gpsData.Time;
						IntermediateRresults[var].CourseTrue = gpsData.CourseTrue;
						memcpy(IntermediateRresults[var].EW, gpsData.EW, sizeof(gpsData.EW));
						memcpy(IntermediateRresults[var].NS, gpsData.NS, sizeof(gpsData.NS));
						IntermediateRresults[var].SLatitude = gpsData.SLatitude;
						IntermediateRresults[var].SLongitude = gpsData.SLongitude;
						IntermediateRresults[var].StatusMeas = MES_FIXED;
						uint32_t resultTime =	gpsData.Time - StartMeas.Time;
						dysplayBufferStruct SymbolBuffer;
						switch (var) {
						case 1:
												getLedBufferFromNumberSpeed(resultTime,&SymbolBuffer);
												SymbolBuffer.ShowDelay = 1000;
												xQueueSendToBack(DyspalyQueueAfterHandle,&SymbolBuffer,5);

												//отобразить результат 5 кмч
												break;
						case 3:
							getLedBufferFromNumberSpeed(resultTime,&SymbolBuffer);
							SymbolBuffer.ShowDelay = 1000;
							xQueueSendToBack(DyspalyQueueAfterHandle,&SymbolBuffer,5);

							//отобразить результат 30 кмч
							break;
						case 6:
							getLedBufferFromNumberSpeed(resultTime,&SymbolBuffer);
							SymbolBuffer.ShowDelay = 1000;
							xQueueSendToBack(DyspalyQueueAfterHandle,&SymbolBuffer,5);
							//отобразить результат 30 кмч
							break;

						case 10:
							getLedBufferFromNumberSpeed(resultTime,&SymbolBuffer);
							SymbolBuffer.ShowDelay = 1000;
							xQueueSendToBack(DyspalyQueueAfterHandle,&SymbolBuffer,5);
							//отобразить результат  кмч
							break;
						case 14:
							getLedBufferFromNumberSpeed(resultTime,&SymbolBuffer);
							SymbolBuffer.ShowDelay = 1000;
							xQueueSendToBack(DyspalyQueueAfterHandle,&SymbolBuffer,5);
							//отобразить результат 140 кмч
							break;

						default:
							break;
						}


					}*/

			}


			/*
				SymbolBuffer.LedState &= ~0b10000000;
			else
				SymbolBuffer.LedState	|= 0b10000000;
			if(gpsData.Status[0]=='A')
				SymbolBuffer.LedState &= ~0b01000000;
			else
				SymbolBuffer.LedState	|= 0b01000000;
			if(gpsData.Speed[0]!='\0')
			{
				uint32_t TempSpeed= AsciiRemoveDotToInt(gpsData.Speed);
				currentSpeed= TempSpeed*1.852;
				TempSpeed = currentSpeed;
				getLedBufferFromNumberSpeed(TempSpeed, &SymbolBuffer);
				xStatus = xQueueSendToBack(DysplayQueue01Handle,&SymbolBuffer,5);*/
		}

	}
}
/* USER CODE END GPSHadlerFunc */


/* DysplayTaskFunc function */
void DysplayTaskFunc(void const * argument)
{
	/* USER CODE BEGIN DysplayTaskFunc */
	dysplayBufferStruct SymbolBuffer;
	portBASE_TYPE xStatus;
	/* Infinite loop */
	for(;;)
	{

		xStatus= xQueueReceive(DyspalyQueueAfterHandle, &SymbolBuffer, 0);
		if(xStatus==pdPASS)
		{
			xStatus = xQueueSendToBack(DysplayQueue01Handle,&SymbolBuffer,5);
			if(SymbolBuffer.ShowDelay!=0) osDelay(SymbolBuffer.ShowDelay);
		}
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
	while(1)
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
