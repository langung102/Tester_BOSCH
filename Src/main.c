/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "software_timer.h"
#include "input_processing.h"
#include "input_reading.h"
#include "stdio.h"
#include "LCD_driver.h"
#include "st7789.h"
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

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader1;
CAN_RxHeaderTypeDef RxHeader1;

uint8_t TxData1[8];
uint8_t RxData1[8];

uint32_t TxMailbox;

int datacheck1 = 0;

int service_2E_state = 0;
int service_27_state = 0;
int cur_button = 0;
char lcd_str[10];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader1 ,RxData1);
	datacheck1 = 1;
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void do_service_22() {
	  if (datacheck1) {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
		  datacheck1 = 0;
		  uint16_t result = (RxData1[3] << 8) | (RxData1[4] << 0);
		  HAL_UART_Transmit(&huart1, (void *)str , sprintf(str, "%d\r\n", result), 1000);

	      sprintf(lcd_str, "ADC received: %d", result);

	      ST7789_Fill_Color(WHITE);
	      ST7789_WriteString(10, 20, lcd_str, Font_11x18, RED, WHITE);
	  } else {
      TxHeader1.DLC   = 3;
      TxHeader1.IDE   = CAN_ID_STD;
      TxHeader1.RTR   = CAN_RTR_DATA;
      TxHeader1.StdId = 0x012;

      TxData1[0] = 0x22;
      TxData1[1] = 0xF0;
      TxData1[2] = 0x01;
		  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox) == HAL_OK){
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		  }
	  }
}

void do_service_2E() {
  switch (service_2E_state) {
    case 0:
      for (cur_button=0; cur_button<5; cur_button++) {
        if (check_button_flag(cur_button)) {
          service_2E_state = 1;
        }
      }
      break;
    case 1:
      if (datacheck1) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
        datacheck1 = 0;
        if (((RxData1[0] >> 4) & 0X0F) == 0x03 && RxData1[1] == (0x2E + 0x40)) {
        	HAL_UART_Transmit(&huart1, "service 2E: Received FC\r\n" , 30, 1000);
        	service_2E_state = 2;
        }
      } else {
        TxHeader1.DLC   = 8;
        TxHeader1.IDE   = CAN_ID_STD;
        TxHeader1.RTR   = CAN_RTR_DATA;
        TxHeader1.StdId = 0x012;

        //PCI
        TxData1[0] = 0x10;
        TxData1[1] = 0x06;
        //SID
        TxData1[2] = 0x2E;
        //DID
        TxData1[3] = 0xF0;
        TxData1[4] = 0x02;
        //Data
        TxData1[5] = (uint8_t) cur_button;
        TxData1[6] = (uint8_t) cur_button;
        TxData1[7] = (uint8_t) cur_button;

        HAL_UART_Transmit(&huart1, "Service 2E: Sending FF\r\n" , 30, 1000);
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox) == HAL_OK){
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
        }
      }
      break;
    case 2:
    	TxHeader1.DLC   = 4;
    	TxHeader1.IDE   = CAN_ID_STD;
    	TxHeader1.RTR   = CAN_RTR_DATA;
    	TxHeader1.StdId = 0x012;

    	//PCI
    	TxData1[0] = ((0x02 << 4) & 0xFF) | 0x01;
    	//DATA
    	TxData1[1] = (uint8_t) cur_button;
    	TxData1[2] = (uint8_t) cur_button;
    	TxData1[3] = (uint8_t) cur_button;

    	HAL_UART_Transmit(&huart1, "Service 2E: Sending CF\r\n" , 30, 1000);
    	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox) == HAL_OK){
    		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
    		service_2E_state = 3;
    	}
    	HAL_Delay(50);
    	break;
    case 3:
    	if (datacheck1) {
            if (((RxData1[0] >> 4) & 0X0F) == 0x03 && RxData1[1] == (0x2E + 0x40)) {
            	HAL_UART_Transmit(&huart1, "Service 2E: Done\r\n" , 30, 1000);
            	service_2E_state = 0;
            }
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    	}
  }
}

void do_service_27() {
	switch (service_27_state) {
		case 0:
			if (datacheck1) {
		        if (RxData1[0] == (0x27 + 0x40) && RxData1[1] == 0x01) {
		        	HAL_UART_Transmit(&huart1, "Service 27: Received seed\r\n" , 30, 1000);
		        	service_27_state = 1;
		        }
		        datacheck1 = 0;
			} else if (check_button_flag(5)) {
		        TxHeader1.DLC   = 2;
		        TxHeader1.IDE   = CAN_ID_STD;
		        TxHeader1.RTR   = CAN_RTR_DATA;
		        TxHeader1.StdId = 0x012;

		        TxData1[0] = 0x27;
		        TxData1[1] = 0x01;

		        HAL_UART_Transmit(&huart1, "Service 27: Request seed\r\n" , 30, 1000);
		        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox) == HAL_OK){
		          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		        }
			}

			break;
		case 1:
	        TxHeader1.DLC   = 2;
	        TxHeader1.IDE   = CAN_ID_STD;
	        TxHeader1.RTR   = CAN_RTR_DATA;
	        TxHeader1.StdId = 0x012;

			TxData1[0] = 0x27;
			TxData1[1] = 0x02;
			TxData1[2] = RxData1[2] + 1;
			TxData1[3] = RxData1[3] + 1;
			TxData1[4] = RxData1[4] + 1;
			TxData1[5] = RxData1[5] + 1;

	        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox) == HAL_OK){
	        	HAL_UART_Transmit(&huart1, "Service 27: Sending key\r\n" , 30, 1000);
	        	service_27_state = 2;
	        }

			break;
		case 2:
			if (datacheck1) {
				if (RxData1[0] == (0x27 + 0x40) && RxData1[1] == 0x02) {
					HAL_UART_Transmit(&huart1, "Service 27: Done\r\n" , 30, 1000);
					service_27_state = 0;
				}
			}
			break;
		default:
			break;
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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  uint8_t service = 0x27;
  lcd_init();
  ST7789_Init();
  ST7789_Fill_Color(WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setTimer0(100);
  while (1)
  {
	  switch (service) {
		case 0x22:
			do_service_22();
			break;
		case 0x2E:
			do_service_2E();
			break;
		case 0x27:
			HAL_UART_Transmit(&huart1, "Service 27: Start\r\n" , 30, 1000);
			do_service_27();
			break;
		default:
			break;
	  }
	  HAL_Delay(20);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 1;  // which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0;
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0;
    canfilterconfig.FilterMaskIdLow = 0;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  CYCLE = 1/((160e6/(htim2.Init.Prescaler + 1))/htim2.Init.Period)*1000;
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LED1_Pin|GPIO_PIN_2|LED3_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : CTR_Pin A_Pin B_Pin C_Pin
                           D_Pin */
  GPIO_InitStruct.Pin = CTR_Pin|A_Pin|B_Pin|C_Pin
                          |D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LED1_Pin PB2 LED3_Pin
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = LED_Pin|LED1_Pin|GPIO_PIN_2|LED3_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == htim2.Instance)
	{
		runTimer();
		button_reading();
	}
}
/* USER CODE END 4 */

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
