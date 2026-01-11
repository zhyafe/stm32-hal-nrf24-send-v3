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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "nrf24.h"
#include "oled.h"
#include "spi.h"
#include "utils.h"
#include <stdint.h>

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
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for nrf24Task */
osThreadId_t nrf24TaskHandle;
const osThreadAttr_t nrf24Task_attributes = {
    .name = "nrf24Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh1,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
    .name = "adcTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
    .name = "oledTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow1,
};
/* Definitions for adcQueue */
osMessageQueueId_t adcQueueHandle;
const osMessageQueueAttr_t adcQueue_attributes = {.name = "adcQueue"};
/* Definitions for oledAdcQueue */
osMessageQueueId_t oledAdcQueueHandle;
const osMessageQueueAttr_t oledAdcQueue_attributes = {.name = "oledAdcQueue"};
/* Definitions for oledSendDataQueue */
osMessageQueueId_t oledSendDataQueueHandle;
const osMessageQueueAttr_t oledSendDataQueue_attributes = {
    .name = "oledSendDataQueue"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartNrf24Task(void *argument);
void StartAdcTask(void *argument);
void StartOledTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of adcQueue */
  adcQueueHandle = osMessageQueueNew(1, 8, &adcQueue_attributes);

  /* creation of oledAdcQueue */
  oledAdcQueueHandle = osMessageQueueNew(1, 8, &oledAdcQueue_attributes);

  /* creation of oledSendDataQueue */
  oledSendDataQueueHandle =
      osMessageQueueNew(1, 10, &oledSendDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of nrf24Task */
  nrf24TaskHandle = osThreadNew(StartNrf24Task, NULL, &nrf24Task_attributes);

  /* creation of adcTask */
  adcTaskHandle = osThreadNew(StartAdcTask, NULL, &adcTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);

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
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  (void)argument;
  for (;;) {
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartNrf24Task */
/**
 * @brief Function implementing the nrf24Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartNrf24Task */
void StartNrf24Task(void *argument) {
  /* USER CODE BEGIN StartNrf24Task */
  /* Infinite loop */
  (void)argument;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
  osStatus_t osStatus;
  uint16_t adcVal[4] = {0, 0, 0, 0};
  uint8_t sendData[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

  // uint16_t temp_data[5];
  // uint8_t sendData[3] = {0, 0, 100};
  // osStatus_t osStatus;
  for (;;) {
    osStatus = osMessageQueueGet(adcQueueHandle, adcVal, NULL, osWaitForever);

    if (osStatus == osOK) {

      for (uint8_t i = 0; i < sizeof(sendData) / sizeof(sendData[0]); i++) {
        if (adcVal[i] >= 2100)
          sendData[i] = mapVal(2100, 4095, 101, 200, adcVal[i]);
        else if (adcVal[i] <= 1900)
          sendData[i] = mapVal(0, 1900, 0, 99, adcVal[i]);
        else
          sendData[i] = 100;
      }
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[4] = 0;
      } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[4] = 200;
      } else {
        sendData[4] = 100;
      }
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[5] = 0;
      } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[5] = 200;
      } else {
        sendData[5] = 100;
      }

      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[6] = 0;
      } else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[6] = 200;
      } else {
        sendData[6] = 100;
      }

      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[7] = 0;
      } else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == RESET) {
        // 如果PB12引脚为低电平，则将发送数据取反
        sendData[7] = 200;
      } else {
        sendData[7] = 100;
      }

      osMessageQueuePut(oledSendDataQueueHandle, sendData, 0, 0);
      osMessageQueuePut(oledAdcQueueHandle, adcVal, 0, 0);

      uint8_t status = NRF24_SendData(sendData, 10);
      if (status == 0) {
        // 发送成功
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      } else if (status == 1) {
        // 发送失败，达到最大重发次数
      } else if (status == 2) {
        // 超时未发送成功
      }
    }

    osDelay(20);
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
void StartAdcTask(void *argument) {
  /* USER CODE BEGIN StartAdcTask */
  /* Infinite loop */
  (void)argument;
  uint16_t adcVal[4] = {0, 0, 0, 0};
  uint16_t old_msg[4] = {0, 0, 0, 0};
  uint16_t queue_count;
  // 校准 ADC
  HAL_ADCEx_Calibration_Start(&hadc1);

  // // 启动第一次 ADC DMA 转换
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal, 4);
  /* Infinite loop */
  for (;;) {

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal, 4);

    // 2. 检查队列当前消息数（非阻塞）
    queue_count = osMessageQueueGetCount(adcQueueHandle);

    // 3. 如果队列已满（有未被读取的旧消息），先清空旧消息
    if (queue_count > 0) {
      // 非阻塞读取旧消息（丢弃，不处理）
      osMessageQueueGet(adcQueueHandle, old_msg, NULL, 0);
    }

    osMessageQueuePut(adcQueueHandle, adcVal, 0, 0);
    // tx_data[0] = 0;

    // uint8_t status = NRF24_SendData(tx_data, 5);
    osDelay(20);
  }
  /* USER CODE END StartAdcTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
 * @brief Function implementing the oledTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument) {
  /* USER CODE BEGIN StartOledTask */
  /* Infinite loop */
  (void)argument;
  uint16_t oledAdcVal[4] = {0, 0, 0, 0};
  uint8_t oledSendDataVal[10] = {0, 0, 0, 0, 0, 0, 0, 0};
  osStatus_t osAdcStatus;
  osStatus_t osSendDataStatus;
  OLED_Init();
  OLED_ShowString(0, 0, "init");
  OLED_Refresh();
  for (;;) {
    OLED_Clear();
    // 阻塞读取消息（永久等待）
    osAdcStatus =
        osMessageQueueGet(oledAdcQueueHandle, oledAdcVal, NULL, osWaitForever);
    if (osAdcStatus == osOK) {
      OLED_ShowNum(0, 0, oledAdcVal[0]);
      OLED_ShowNum(30, 0, oledAdcVal[1]);
      OLED_ShowNum(60, 0, oledAdcVal[2]);
      OLED_ShowNum(90, 0, oledAdcVal[3]);
    }

    osSendDataStatus = osMessageQueueGet(oledSendDataQueueHandle,
                                         oledSendDataVal, NULL, osWaitForever);
    if (osSendDataStatus == osOK) {

      OLED_ShowNum(0, 1, oledSendDataVal[0]);
      OLED_ShowNum(30, 1, oledSendDataVal[1]);
      OLED_ShowNum(60, 1, oledSendDataVal[2]);
      OLED_ShowNum(90, 1, oledSendDataVal[3]);
    }
    OLED_Refresh();
    osDelay(100);
  }
  /* USER CODE END StartOledTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
