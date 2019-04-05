/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "can.h"
#include "can_task.h"
#include "inv_mpu.h"
#include "imu_task.h"
#include "Init_Task.h"
#include "print_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
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
osThreadId InitTaskHandle;
osThreadId ControlTaskHandle;
osThreadId IMUTaskHandle;
osThreadId ShootTaskHandle;
osTimerId CanTimerSendHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartInitTask(void const * argument);
void StartControlTask(void const * argument);
void StartIMUTask(void const * argument);
void StartShootTask(void const * argument);
void CanTimerSendCallback(void const * argument);

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

  /* Create the timer(s) */
  /* definition and creation of CanTimerSend */
  osTimerDef(CanTimerSend, CanTimerSendCallback);
  CanTimerSendHandle = osTimerCreate(osTimer(CanTimerSend), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of InitTask */
  osThreadDef(InitTask, StartInitTask, osPriorityHigh, 0, 512);
  InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityBelowNormal, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, StartIMUTask, osPriorityNormal, 0, 256);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, StartShootTask, osPriorityIdle, 0, 256);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartInitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartInitTask */
void StartInitTask(void const * argument)
{

  /* USER CODE BEGIN StartInitTask */
  /* Infinite loop */
		bsp_init ();
		my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
		HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
	  osTimerStart(CanTimerSendHandle, 1);
		vTaskDelete (InitTaskHandle);
  /* USER CODE END StartInitTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
		gimbal_change();
    vTaskDelay (1);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void const * argument)
{
  /* USER CODE BEGIN StartIMUTask */
	portTickType CurrentControlTick = 0;                //当前系统时间
  /* Infinite loop */
  for(;;)
  {
    CurrentControlTick = xTaskGetTickCount();
    Get_IMU_Data();
    vTaskDelayUntil(&CurrentControlTick, 5 / portTICK_RATE_MS);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShootTask */
void StartShootTask(void const * argument)
{
  /* USER CODE BEGIN StartShootTask */
  /* Infinite loop */
  for(;;)
  {
		shoot_mode();
		shanwai_sprintf();
		vTaskDelay (2);
  }
  /* USER CODE END StartShootTask */
}

/* CanTimerSendCallback function */
void CanTimerSendCallback(void const * argument)
{
  /* USER CODE BEGIN CanTimerSendCallback */
  GIMBAL_CAN_SEND();
  /* USER CODE END CanTimerSendCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
