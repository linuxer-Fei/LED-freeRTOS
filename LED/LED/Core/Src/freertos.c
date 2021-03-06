/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "stdio.h"
#include "debug_tool.h"
#include "mpu6050.h"
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  extern int keyIntOccered;
  struct db_data gyro_x = {
		.chl = CHANNEL_0,
		.isFloat = 1,
  };
  struct db_data gyro_y = {
		.chl = CHANNEL_1,
		.isFloat = 1,
  };
  struct db_data gyro_z = {
		.chl = CHANNEL_2,
		.isFloat = 1,
  };
  struct db_data acc_x = {
		.chl = CHANNEL_3,
		.isFloat = 1,
  };
  struct db_data acc_y = {
		.chl = CHANNEL_4,
		.isFloat = 1,
  };
  struct db_data acc_z = {
		.chl = CHANNEL_5,
		.isFloat = 1,
  };
  db_creat(&gyro_x);
  db_creat(&gyro_y);
  db_creat(&gyro_z);
  db_creat(&acc_x);
  db_creat(&acc_y);
  db_creat(&acc_z);
  struct gyro_acc_data data;
  for(;;)
  {
	  if (keyIntOccered) {
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		  //OS_LOG(HIGH, "s \r\n");
		  mpu6050_get_gyro_acc_data(&data);
		  OS_LOG(LOW, "[%f, %f, %f], [%f, %f, %f] \r\n",
		  	  data.gyro_x, data.gyro_y, data.gyro_z,
		  	  data.acc_x, data.acc_y,data.acc_z);
#if 1
		  db_data_update(&gyro_x, &data.gyro_x);
		  db_data_update(&gyro_y, &data.gyro_y);
		  db_data_update(&gyro_z, &data.gyro_z);
		  db_data_update(&acc_x, &data.acc_x);
		  db_data_update(&acc_y, &data.acc_y);
		  db_data_update(&acc_z, &data.acc_z);
		  db_show_data();
#endif
		  osDelay(10);
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  } else {
		  osDelay(1);
	  }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

