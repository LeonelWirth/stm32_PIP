/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "Modbus.h"
#include "ModbusConfig.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* Definitions for Modbus */
osThreadId_t ModbusHandle;
const osThreadAttr_t Modbus_attributes = {
  .name = "Modbus",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADC */
osThreadId_t ADCHandle;
const osThreadAttr_t ADC_attributes = {
  .name = "ADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Encoders */
osThreadId_t EncodersHandle;
const osThreadAttr_t Encoders_attributes = {
  .name = "Encoders",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Control */
osThreadId_t ControlHandle;
const osThreadAttr_t Control_attributes = {
  .name = "Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for QueueDataADC */
osMessageQueueId_t QueueDataADCHandle;
const osMessageQueueAttr_t QueueDataADC_attributes = {
  .name = "QueueDataADC"
};
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StartModbus(void *argument);
void StartADC(void *argument);
void StartEncoders(void *argument);
void StartControl(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//---------------->  Modbus
modbusHandler_t ModbusH;
uint16_t ModbusDATA[13]={1,8000,0,0,0,0,0,0,0,0,0,0,0}; // Mapa modbus!
//---------------->
uint32_t Ts =100; // En ms!
float incremento_enconder = 0;
float velocidad;
float ranuras = 20.0;
float freq = 10.0;

//Si se interrumpe por flanco ascendente del pin 0 (Enconder optico)
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if (GPIO_Pin == D01_Encoder_Pin){
		incremento_enconder += 1;
	}
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void Sentido(uint16_t valor){
	//Motor gira en un sentido
	if(valor == 0){
		HAL_GPIO_WritePin(OUT1_2_GPIO_Port, OUT1_2_Pin, SET);
		HAL_GPIO_WritePin(OUT1_1_GPIO_Port, OUT1_1_Pin, RESET);
	}
	//Motor gira en otro sentido
	else if(valor == 1){
		HAL_GPIO_WritePin(OUT1_2_GPIO_Port, OUT1_2_Pin, RESET);
		HAL_GPIO_WritePin(OUT1_1_GPIO_Port, OUT1_1_Pin, SET);

	}
	else{ // Break
		HAL_GPIO_WritePin(OUT1_2_GPIO_Port, OUT1_2_Pin, RESET);
		HAL_GPIO_WritePin(OUT1_1_GPIO_Port, OUT1_1_Pin, RESET);
	}
}

void Velocidad(uint16_t param){
	if(incremento_enconder == 0){
		velocidad = 0.0;
	}else{
		if (param ==0){
			velocidad = freq*incremento_enconder/(ranuras);
			incremento_enconder=0;
		}
		if (param == 1){
			velocidad =  freq*incremento_enconder/(ranuras);
			incremento_enconder=0;
		}
	}
}










/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Definiciones para la biblioteca de modbus
   ModbusH.uModbusType = MB_SLAVE;
   ModbusH.port =  &huart3;
   ModbusH.u8id = 1; //Modbus slave ID
   ModbusH.u16timeOut = 1000;
   ModbusH.EN_Port = NULL;
   ModbusH.u16regs = ModbusDATA;
   ModbusH.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
   ModbusH.xTypeHW = USART_HW;

   //Initialize Modbus library
   ModbusInit(&ModbusH);
   //Start capturing traffic on serial Port
   ModbusStart(&ModbusH);

   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueDataADC */
  QueueDataADCHandle = osMessageQueueNew (16, sizeof(uint16_t), &QueueDataADC_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  if ( QueueDataADCHandle == 0)  // Queue not created
  {
	  char *str = "Unable to create Integer Queue\n\n";
//	  HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
  }
  else
  {
	  char *str = "Integer Queue Created successfully\n\n";
//	  HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Modbus */
  ModbusHandle = osThreadNew(StartModbus, NULL, &Modbus_attributes);

  /* creation of ADC */
  ADCHandle = osThreadNew(StartADC, NULL, &ADC_attributes);

  /* creation of Encoders */
  EncodersHandle = osThreadNew(StartEncoders, NULL, &Encoders_attributes);

  /* creation of Control */
  ControlHandle = osThreadNew(StartControl, NULL, &Control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT2_1_Pin|OUT2_2_Pin|OUT1_2_Pin|OUT1_1_Pin
                          |OUT3_2_Pin|OUT3_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT4_1_Pin|OUT4_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT2_1_Pin OUT2_2_Pin OUT1_2_Pin OUT1_1_Pin
                           OUT3_2_Pin OUT3_1_Pin */
  GPIO_InitStruct.Pin = OUT2_1_Pin|OUT2_2_Pin|OUT1_2_Pin|OUT1_1_Pin
                          |OUT3_2_Pin|OUT3_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT4_1_Pin OUT4_2_Pin */
  GPIO_InitStruct.Pin = OUT4_1_Pin|OUT4_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D01_Encoder_Pin D02_Encoder_Pin D03_Encoder_Pin D04_Encoder_Pin */
  GPIO_InitStruct.Pin = D01_Encoder_Pin|D02_Encoder_Pin|D03_Encoder_Pin|D04_Encoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartModbus */
/**
  * @brief  Function implementing the Modbus thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartModbus */
void StartModbus(void *argument)
{
  /* USER CODE BEGIN 5 */
	int i =0;
	char buff[64];
	uint16_t valor =1234;
	char *prt;
	osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
//	  osMessageGet( QueueDataADCHandle, &valor, 0 );
	prt=pvPortMalloc(100*sizeof (char));
	sprintf(prt,"Dato leido de la Queue: %u \n",valor);

	 status = osMessageQueueGet(QueueDataADCHandle, &valor, NULL, 5000);   // wait for message
	    if (status == osOK) {
//	HAL_UART_Transmit(&huart3, (uint8_t*)prt, strlen(prt), 100);
	vPortFree(prt);
	      ; // process data
	    }

    osDelay(900);
//    ModbusDATA[5]= ++i;

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartADC */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC */
void StartADC(void *argument)
{
  /* USER CODE BEGIN StartADC */

	uint16_t adc1[4];
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1,sizeof (adc1));
//	HAL_ADC_Start_DMA(hadc, pData, Length)
  /* Infinite loop */
  for(;;)
  {
//	HAL_ADC_Stop_DMA(&hadc1);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1,sizeof (adc1));
//	adc1 = HAL_ADC_PollForConversion(&hadc1, 5000);
//	osMessageQueuePut(QueueDataADCHandle, &adc1, 5000);
	osMessageQueuePut(QueueDataADCHandle, &adc1[0], NULL, 5000);
	osThreadYield();

    osDelay(1000);
  }
  /* USER CODE END StartADC */
}

/* USER CODE BEGIN Header_StartEncoders */
/**
* @brief Function implementing the Encoders thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoders */
void StartEncoders(void *argument)
{
  /* USER CODE BEGIN StartEncoders */
	// Signo esta en ModbusDATA[0]
	// Setpoint esta enModbusDATA[1]
	// Velocidad es global

  /* Infinite loop */
  for(;;)
  {

    Velocidad(ModbusDATA[0]);// Calculo la velocidad para devolver por modbus
    osDelay(Ts);// Delta T
    Sentido(ModbusDATA[0]);
    ModbusDATA[8] = (uint16_t)(100.0*velocidad);
    ModbusDATA[9] = (uint16_t)incremento_enconder;
    htim1.Instance->CCR1 = ModbusDATA[1];


  }
  /* USER CODE END StartEncoders */
}

/* USER CODE BEGIN Header_StartControl */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl */
void StartControl(void *argument)
{
  /* USER CODE BEGIN StartControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartControl */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  // ESTO ES DE FREERTOS
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
