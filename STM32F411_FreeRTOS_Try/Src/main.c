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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId myTaskIdle_0Handle;
osThreadId myTaskLow_1Handle;
osThreadId myTaskBNormal_2Handle;
osThreadId myTaskNormal_3Handle;
osThreadId myTaskANormal_4Handle;
osThreadId myTaskHigh_5Handle;
osThreadId myTaskRT_6Handle;
osMessageQId myQueueU8Handle;
osMessageQId myQueueU16Handle;
osTimerId myTimerAutoReloadHandle;
osTimerId myTimerOneShotHandle;
osMutexId myMutex_1Handle;
osMutexId myMutex_2Handle;
osMutexId myRecursiveMutex_1Handle;
osMutexId myRecursiveMutex_2Handle;
osSemaphoreId myBinarySem_1Handle;
osSemaphoreId myBinarySem_2Handle;
osSemaphoreId myCountingSem_1Handle;
osSemaphoreId myCountingSem_2Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_TxBuffLen  0x20
#define UART_RxBuffLen  0x08

uint8_t UART_N1_TxBuffer[UART_TxBuffLen];
uint8_t UART_N1_RxBuffer[UART_RxBuffLen];
uint8_t UART_N2_TxBuffer[UART_TxBuffLen];
uint8_t UART_N2_RxBuffer[UART_RxBuffLen];


#define KEY_BUTTON_PIN  (0x1U << 13)

//------------------------------
#define EVENTBIT_0		(1<<0)
#define EVENTBIT_1		(1<<1)
#define EVENTBIT_2		(1<<2)
#define EVENTBIT_ALL   (EVENTBIT_1|EVENTBIT_2)

EventGroupHandle_t EventGroupHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void StartTaskIdle_0(void const * argument);
void StartTaskLow_1(void const * argument);
void StartTaskBelowNormal_2(void const * argument);
void StartTaskNormal_3(void const * argument);
void StartTaskAboveNormal_4(void const * argument);
void StartTaskHigh_5(void const * argument);
void StartTaskRealtime_6(void const * argument);
void TimerAutoReloadCallback(void const * argument);
void TimerOneShotCallback(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t GetlengthChar(char* chararray) {
#if 1
    uint8_t n = 0;
    while(chararray[n] != '\0')
    {
        n++;
        if(n >= UART_TxBuffLen) return 0;
    }
    return n;
#else
    uint8_t len;
    for(len = 0; *chararray++ != '\0'; len++);
    return len;
#endif
}

void printf_to_str(char *str)
{
    uint16_t leng;
    leng = GetlengthChar(str);
    HAL_UART_Transmit(&huart1,(uint8_t *)str,leng,1);
}

void printMegln(char *fmt,...)
{
    va_list ap;
    char str[UART_TxBuffLen];
    char strln[] = {'\x0D','\x0A','\0'};

    va_start(ap,fmt);
    vsprintf(str,fmt,ap);
    va_end(ap);

    strcat(str, strln);
    printf_to_str(str);
}

void printMeg(char *fmt,...)
{
    va_list ap;
    char str[UART_TxBuffLen];

    va_start(ap,fmt);
    vsprintf(str,fmt,ap);
    va_end(ap);

    printf_to_str(str);
}

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, UART_N1_RxBuffer, 1);
  printMegln("STM32F411 FreeRTOS");


  EventGroupHandle = xEventGroupCreate();
  if(EventGroupHandle == NULL) {
    printMegln("Event Group Create Failed!\r\n");
  }
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex_1 */
  osMutexDef(myMutex_1);
  myMutex_1Handle = osMutexCreate(osMutex(myMutex_1));

  /* definition and creation of myMutex_2 */
  osMutexDef(myMutex_2);
  myMutex_2Handle = osMutexCreate(osMutex(myMutex_2));

  /* Create the recursive mutex(es) */
  /* definition and creation of myRecursiveMutex_1 */
  osMutexDef(myRecursiveMutex_1);
  myRecursiveMutex_1Handle = osRecursiveMutexCreate(osMutex(myRecursiveMutex_1));

  /* definition and creation of myRecursiveMutex_2 */
  osMutexDef(myRecursiveMutex_2);
  myRecursiveMutex_2Handle = osRecursiveMutexCreate(osMutex(myRecursiveMutex_2));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem_1 */
  osSemaphoreDef(myBinarySem_1);
  myBinarySem_1Handle = osSemaphoreCreate(osSemaphore(myBinarySem_1), 1);

  /* definition and creation of myBinarySem_2 */
  osSemaphoreDef(myBinarySem_2);
  myBinarySem_2Handle = osSemaphoreCreate(osSemaphore(myBinarySem_2), 1);

  /* definition and creation of myCountingSem_1 */
  osSemaphoreDef(myCountingSem_1);
  myCountingSem_1Handle = osSemaphoreCreate(osSemaphore(myCountingSem_1), 10);

  /* definition and creation of myCountingSem_2 */
  osSemaphoreDef(myCountingSem_2);
  myCountingSem_2Handle = osSemaphoreCreate(osSemaphore(myCountingSem_2), 20);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimerAutoReload */
  osTimerDef(myTimerAutoReload, TimerAutoReloadCallback);
  myTimerAutoReloadHandle = osTimerCreate(osTimer(myTimerAutoReload), osTimerPeriodic, NULL);

  /* definition and creation of myTimerOneShot */
  osTimerDef(myTimerOneShot, TimerOneShotCallback);
  myTimerOneShotHandle = osTimerCreate(osTimer(myTimerOneShot), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(myTimerAutoReloadHandle, 5000);
  osTimerStart(myTimerOneShotHandle, 1000);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of myTaskIdle_0 */
  osThreadDef(myTaskIdle_0, StartTaskIdle_0, osPriorityIdle, 0, 128);
  myTaskIdle_0Handle = osThreadCreate(osThread(myTaskIdle_0), NULL);

  /* definition and creation of myTaskLow_1 */
  osThreadDef(myTaskLow_1, StartTaskLow_1, osPriorityLow, 0, 128);
  myTaskLow_1Handle = osThreadCreate(osThread(myTaskLow_1), NULL);

  /* definition and creation of myTaskBNormal_2 */
  osThreadDef(myTaskBNormal_2, StartTaskBelowNormal_2, osPriorityBelowNormal, 0, 128);
  myTaskBNormal_2Handle = osThreadCreate(osThread(myTaskBNormal_2), NULL);

  /* definition and creation of myTaskNormal_3 */
  osThreadDef(myTaskNormal_3, StartTaskNormal_3, osPriorityNormal, 0, 128);
  myTaskNormal_3Handle = osThreadCreate(osThread(myTaskNormal_3), NULL);

  /* definition and creation of myTaskANormal_4 */
  osThreadDef(myTaskANormal_4, StartTaskAboveNormal_4, osPriorityAboveNormal, 0, 128);
  myTaskANormal_4Handle = osThreadCreate(osThread(myTaskANormal_4), NULL);

  /* definition and creation of myTaskHigh_5 */
  osThreadDef(myTaskHigh_5, StartTaskHigh_5, osPriorityHigh, 0, 128);
  myTaskHigh_5Handle = osThreadCreate(osThread(myTaskHigh_5), NULL);

  /* definition and creation of myTaskRT_6 */
  osThreadDef(myTaskRT_6, StartTaskRealtime_6, osPriorityRealtime, 0, 128);
  myTaskRT_6Handle = osThreadCreate(osThread(myTaskRT_6), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of myQueueU8 */
  osMessageQDef(myQueueU8, 16, uint8_t);
  myQueueU8Handle = osMessageCreate(osMessageQ(myQueueU8), NULL);

  /* definition and creation of myQueueU16 */
  osMessageQDef(myQueueU16, 16, uint16_t);
  myQueueU16Handle = osMessageCreate(osMessageQ(myQueueU16), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        //printMeg("%c",UART_N1_RxBuffer[0]);
        if(osMessagePut(myQueueU8Handle, UART_N1_RxBuffer[0], 0) != osOK) {
            printMeg("!UR1",UART_N1_RxBuffer[0]);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
      //printMegln("%#x",GPIO_Pin);
      if(osSemaphoreRelease(myBinarySem_1Handle) != osOK) {
        printMegln("GPIO EXTI BinarySem: %#x",GPIO_Pin);
      }

      if(osSemaphoreRelease(myCountingSem_1Handle) != osOK) {
        printMegln("GPIO EXTI CountingSem: %#x",GPIO_Pin);
      }

      if(osTimerStart(myTimerOneShotHandle, 1000) != osOK) {
        printMegln("GPIO EXTI Time One Shot: %#x",GPIO_Pin);
      }
  }
}

/* USER CODE END 4 */

/* StartTaskIdle_0 function */
void StartTaskIdle_0(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* StartTaskLow_1 function */
void StartTaskLow_1(void const * argument)
{
  /* USER CODE BEGIN StartTaskLow_1 */
  static uint32_t times;
  /* Infinite loop */
  for(;;)
  {
    xEventGroupSetBits(EventGroupHandle, EVENTBIT_1);
    osMutexWait(myMutex_1Handle, osWaitForever);
    printMegln("lwo task running!");
    for(times=0; times < 2000000; times++) {
        //osDelay(10);
        osThreadYield();
    }
    printMegln("lwo task end!");
    osMutexRelease(myMutex_1Handle);
    osDelay(1000);
  }
  /* USER CODE END StartTaskLow_1 */
}

/* StartTaskBelowNormal_2 function */
void StartTaskBelowNormal_2(void const * argument)
{
  /* USER CODE BEGIN StartTaskBelowNormal_2 */
  /* Infinite loop */
  for(;;)
  {
    xEventGroupSetBits(EventGroupHandle, EVENTBIT_2);
    printMegln("Below Normal task running!");
    osDelay(1000);
  }
  /* USER CODE END StartTaskBelowNormal_2 */
}

/* StartTaskNormal_3 function */
void StartTaskNormal_3(void const * argument)
{
  /* USER CODE BEGIN StartTaskNormal_3 */
  /* Infinite loop */
  for(;;)
  {
	printMegln("Normal task Pend Sem!");
	osMutexWait(myMutex_1Handle, osWaitForever);
	printMegln("Normal task running!");
	osMutexRelease(myMutex_1Handle);
    osDelay(1000);
  }
  /* USER CODE END StartTaskNormal_3 */
}

/* StartTaskAboveNormal_4 function */
void StartTaskAboveNormal_4(void const * argument)
{
  /* USER CODE BEGIN StartTaskAboveNormal_4 */
  //osSemaphoreId semaphore = (osSemaphoreId) argument;
  /* Infinite loop */
  for(;;)
  {
    if (myCountingSem_1Handle != NULL) {
      /* Try to obtain the semaphore. */
      if(osSemaphoreWait(myCountingSem_1Handle , 0) == osOK) {
        printMegln("Button CountingSem: %d",osSemaphoreGetCount(myCountingSem_1Handle));
      }
    }
    osDelay(100);
  }
  /* USER CODE END StartTaskAboveNormal_4 */
}

/* StartTaskHigh_5 function */
void StartTaskHigh_5(void const * argument)
{
  /* USER CODE BEGIN StartTaskHigh_5 */
  osEvent event1;
  /* Infinite loop */
  for(;;)
  {
    //printMegln("StartTaskHigh_5");
    if (myQueueU8Handle != NULL) {
        event1 = osMessageGet(myQueueU8Handle, 0);
        if(event1.status == osEventMessage) {
            //AL_UART_Transmit_IT(&huart1, (uint8_t *)&event1.value.v, 1);
            printMeg("%c",event1.value.v);
        }
    }
    if (myBinarySem_1Handle != NULL) {
      /* Try to obtain the semaphore. */
      if(osSemaphoreWait(myBinarySem_1Handle , 0) == osOK) {
        printMegln("Button INT");
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartTaskHigh_5 */
}

/* StartTaskRealtime_6 function */
void StartTaskRealtime_6(void const * argument)
{
  /* USER CODE BEGIN StartTaskRealtime_6 */
  EventBits_t EventValue;
  /* Infinite loop */
  for(;;)
  {
	if(EventGroupHandle != NULL) {
		EventValue = xEventGroupWaitBits(EventGroupHandle,
								         EVENTBIT_ALL,
								         pdTRUE,
							             pdFALSE,
								         portMAX_DELAY);
		printMegln("EventValue = %#x",EventValue);
	}
    osDelay(100);
  }
  /* USER CODE END StartTaskRealtime_6 */
}

/* TimerAutoReloadCallback function */
void TimerAutoReloadCallback(void const * argument)
{
  /* USER CODE BEGIN TimerAutoReloadCallback */
  printMegln("Timer Auto Reload!");

  /* USER CODE END TimerAutoReloadCallback */
}

/* TimerOneShotCallback function */
void TimerOneShotCallback(void const * argument)
{
  /* USER CODE BEGIN TimerOneShotCallback */
  printMegln("Timer One Shot!");
  /* USER CODE END TimerOneShotCallback */
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
