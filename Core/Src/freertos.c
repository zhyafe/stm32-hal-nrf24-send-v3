/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "key.h"
#include "nrf24.h"
#include "spi.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for keyPressTask */
osThreadId_t keyPressTaskHandle;
const osThreadAttr_t keyPressTask_attributes = {
  .name = "keyPressTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for nrf24Task */
osThreadId_t nrf24TaskHandle;
const osThreadAttr_t nrf24Task_attributes = {
  .name = "nrf24Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartKeyPressTask(void *argument);
void StartNrf24Task(void *argument);
void StartAdcTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of keyPressTask */
  keyPressTaskHandle = osThreadNew(StartKeyPressTask, NULL, &keyPressTask_attributes);

  /* creation of nrf24Task */
  nrf24TaskHandle = osThreadNew(StartNrf24Task, NULL, &nrf24Task_attributes);

  /* creation of adcTask */
  adcTaskHandle = osThreadNew(StartAdcTask, NULL, &adcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  (void) argument;
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartKeyPressTask */
/**
* @brief Function implementing the keyPressTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyPressTask */
void StartKeyPressTask(void *argument)
{
  /* USER CODE BEGIN StartKeyPressTask */
  /* Infinite loop */
  (void)argument;
  for(;;)
  {
    // if(getKeyPressFlag(GPIOB, GPIO_PIN_0) == 1){
    //   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // }
    // if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET){
    //   // Placeholder for NRF24 send function
    //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
    // }else{
    //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
    // }
    osDelay(1);
  }
  /* USER CODE END StartKeyPressTask */
}

/* USER CODE BEGIN Header_StartNrf24Task */
/**
* @brief Function implementing the nrf24Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNrf24Task */
void StartNrf24Task(void *argument)
{
  /* USER CODE BEGIN StartNrf24Task */
  /* Infinite loop */
  (void)argument;
   // 初始化NRF24L01
  NRF24_Init(&hspi1);

  // 检测NRF24L01是否存在
  if (NRF24_Check() != 1) {
    // 未检测到NRF24L01，可以在这里添加错误处理
    while (1)
      ;
  }
  // 设置为发送模式
  NRF24_SetTxMode(nrf24_addr);

  uint16_t temp_data[5];
  uint8_t sendData[3] = {0,0,100};
  osStatus_t osStatus;
  for(;;)
  {
    
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET){
      sendData[0] = 100;
    }else{
      sendData[0] = 0;
    }
      
      
    uint8_t status = NRF24_SendData(sendData, 3);
    osDelay(1);
  }
  /* USER CODE END StartNrf24Task */
}

/* USER CODE BEGIN Header_StartAdcTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void *argument)
{
  /* USER CODE BEGIN StartAdcTask */
  /* Infinite loop */
  (void)argument;
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartAdcTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

