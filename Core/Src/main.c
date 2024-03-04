/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

osThreadId CAN1Handle;
osThreadId CAN2Handle;
osThreadId CAN3Handle;
osThreadId CAN4Handle;
osSemaphoreId BinSemHandle;
/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];

uint32_t TxMailBox;
uint8_t x=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
void CanLowPriorityTask(void const * argument);
void CanNormalPriorityTask(void const * argument);
void CANAboveNormalPriorityTask(void const * argument);
void CANHighPriorityTask(void const * argument);

/* USER CODE BEGIN PFP */
void printmsg(char *msg);
void CAN_HP(void);
void CAN_LP(void);
void CAN_NP(void);
void CAN_ANP(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */



HAL_CAN_Start(&hcan1);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinSem */
  osSemaphoreDef(BinSem);
  BinSemHandle = osSemaphoreCreate(osSemaphore(BinSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CAN1 */
  osThreadDef(CAN1, CanLowPriorityTask, osPriorityLow, 0, 128);
  CAN1Handle = osThreadCreate(osThread(CAN1), NULL);

  /* definition and creation of CAN2 */
  osThreadDef(CAN2, CanNormalPriorityTask, osPriorityNormal, 0, 128);
  CAN2Handle = osThreadCreate(osThread(CAN2), NULL);

  /* definition and creation of CAN3 */
  osThreadDef(CAN3, CANAboveNormalPriorityTask, osPriorityAboveNormal, 0, 128);
  CAN3Handle = osThreadCreate(osThread(CAN3), NULL);

  /* definition and creation of CAN4 */
  osThreadDef(CAN4, CANHighPriorityTask, osPriorityHigh, 0, 128);
  CAN4Handle = osThreadCreate(osThread(CAN4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
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

  //configure CAN peripheral

 CAN_FilterTypeDef FilterConfig;

 FilterConfig.FilterBank = 10;
 FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
 FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
 FilterConfig.FilterIdHigh = 0x18FD;
 FilterConfig.FilterIdLow = 0xB7FE<<3;
 FilterConfig.FilterMaskIdHigh = 0xFFFF;
 FilterConfig.FilterMaskIdLow = 0xFFFF<<3;
 FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
 FilterConfig.FilterActivation = ENABLE;
 FilterConfig.SlaveStartFilterBank = 14;

 HAL_CAN_ConfigFilter(&hcan1, &FilterConfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void printmsg( char *msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)msg,strlen(msg),1000);
}


void CAN_HP(void)
{
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.ExtId = 0x18FDB7FE;
	TxHeader.RTR = CAN_RTR_DATA ;
	TxHeader.DLC = 8;

	TxData[0]=x;
	TxData[1]=0x2A;
	TxData[2]=0x30;
	TxData[3]=0x4A;
	TxData[4]=0x50;
	TxData[5]=0x6A;
	TxData[6]=0x70;
	TxData[7]=0x8A;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);

}



void CAN_ANP(void)
{
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = 0x18FDB7FE;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8;

		TxData[0]=x;
		TxData[1]=0xA2;
		TxData[2]=0x53;
		TxData[3]=0xA4;
		TxData[4]=0x55;
		TxData[5]=0xA6;
		TxData[6]=0x57;
		TxData[7]=0xA8;

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
}



void CAN_NP(void)
{
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = 0x18FDB7FE;
		TxHeader.RTR = CAN_RTR_DATA ;
		TxHeader.DLC = 8;

		TxData[0]=x;
		TxData[1]=0xA8;
		TxData[2]=0x57;
		TxData[3]=0xA6;
		TxData[4]=0x10;
		TxData[5]=0x2A;
		TxData[6]=0x30;
		TxData[7]=0x4A;

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
}


void CAN_LP(void)
{
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = 0x18FDB7FE;
		TxHeader.RTR = CAN_RTR_DATA ;
		TxHeader.DLC = 8;

		TxData[0]=x;
		TxData[1]=0xA6;
		TxData[2]=0x30;
		TxData[3]=0xA6;
		TxData[4]=0x20;
		TxData[5]=0xA6;
		TxData[6]=0x52;
		TxData[7]=0xA0;

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_CanLowPriorityTask */
/**
  * @brief  Function implementing the CAN1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CanLowPriorityTask */
void CanLowPriorityTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(BinSemHandle, osWaitForever);
	  	  	  /*char *str1="entered low priority CAN task\n\r";
	  	  	  printmsg(str1);
	  		  char *str2 = "semaphore acquired by CAN Low task\n\r";
	  		  printmsg(str2);*/

	  		  CAN_LP();

	  	      osSemaphoreRelease(BinSemHandle);
	  	      //char *str3 = "leaving low priority CAN task\n\r";
	  	      //printmsg(str3);
	  	      osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_CanNormalPriorityTask */
/**
* @brief Function implementing the CAN2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanNormalPriorityTask */
void CanNormalPriorityTask(void const * argument)
{
  /* USER CODE BEGIN CanNormalPriorityTask */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(BinSemHandle, osWaitForever);
	  	  /*char *str1="entered normal priority CAN task\n\r";
	  	  printmsg(str1);
	  		  char *str2 = "semaphore acquired by CAN High task\n\r";
	  		  printmsg(str2);*/

	  		  CAN_NP();

	  	      osSemaphoreRelease(BinSemHandle);

	  	     // char *str3 = "leaving normal priority CAN task\n\r";
	  	      //printmsg(str3);
	  	      osDelay(50);
  }
  /* USER CODE END CanNormalPriorityTask */
}

/* USER CODE BEGIN Header_CANAboveNormalPriorityTask */
/**
* @brief Function implementing the CAN3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CANAboveNormalPriorityTask */
void CANAboveNormalPriorityTask(void const * argument)
{
  /* USER CODE BEGIN CANAboveNormalPriorityTask */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(BinSemHandle, osWaitForever);
	  /*char *str1="entered above normal priority CAN task\n\r";
	  	  	  printmsg(str1);
        char *str2 = "semaphore acquired by CAN High task\n\r";
	  		  printmsg(str2);*/

	  		  CAN_ANP();

	  	      osSemaphoreRelease(BinSemHandle);
	  	    // char *str3 = "leaving above normal priority CAN task\n\r";
	  	      //printmsg(str3);
	  	      osDelay(50);
  }
  /* USER CODE END CANAboveNormalPriorityTask */
}

/* USER CODE BEGIN Header_CANHighPriorityTask */
/**
* @brief Function implementing the CAN4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CANHighPriorityTask */
void CANHighPriorityTask(void const * argument)
{
  /* USER CODE BEGIN CANHighPriorityTask */
  /* Infinite loop */
  for(;;)
  {

	  osSemaphoreWait(BinSemHandle, osWaitForever);
	  /*char *str1="entered high priority CAN task\n\r";
	  printmsg(str1);


	  char *str2 = "semaphore acquired by CAN High task\n\r";
	  printmsg(str2);*/

	  CAN_HP();

      osSemaphoreRelease(BinSemHandle);
	 //char *str3 = "leaving high priority CAN task\n\r";
	  //printmsg(str3);
    osDelay(50);
  }
  /* USER CODE END CANHighPriorityTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
