/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_freertos.h"
#include "WifiTask.h"
#include "WifiApp.h"
#include "SDCard.h"
#include "CANLogger.h"
#include "Uploader.h"
#include "stm32h5xx_nucleo.h"
#include "WifiTask.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define EVT_HAS_IP         (1U << 0)
#define EVT_BTN_PRESSED    (1U << 1)
#define EVT_CANLOG_START   (1U << 2)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* RTOS objects shared across tasks */
osEventFlagsId_t g_sysEvt;
osMutexId_t      g_sdMutex;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void DebounceButtonAndSignal(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for WifiTask */
osThreadId_t WifiTaskHandle;
const osThreadAttr_t WifiTask_attributes = {
  .name = "WifiTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 3072
};
/* Definitions for CANLogTask */
osThreadId_t CANLogTaskHandle;
const osThreadAttr_t CANLogTask_attributes = {
  .name = "CANLogTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024
};
/* Definitions for UploadTask */
osThreadId_t UploadTaskHandle;
const osThreadAttr_t UploadTask_attributes = {
  .name = "UploadTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 1024
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void DebounceButtonAndSignal(void)
{
  static uint32_t lastTick = 0;
  static uint8_t  lastState = 1; // Nucleo button idle = released = 1
  uint8_t s = BSP_PB_GetState(BUTTON_USER);
  uint32_t now = osKernelGetTickCount();
  if (s == 0 && lastState == 1 && (now - lastTick) > 200) { // falling edge + 200ms
    osEventFlagsSet(g_sysEvt, EVT_BTN_PRESSED);
    lastTick = now;
  }
  lastState = s;
}
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	  const osMutexAttr_t sd_mutex_attr = { .name = "sdMutex" };
	  g_sdMutex = osMutexNew(&sd_mutex_attr);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  printf("heap free=%lu min=%lu\r\n",
         (unsigned long)xPortGetFreeHeapSize(),
         (unsigned long)xPortGetMinimumEverFreeHeapSize());
  /* creation of WifiTask */
  WifiTaskHandle = osThreadNew(StartTask02, NULL, &WifiTask_attributes);
  printf("heap free=%lu min=%lu\r\n",
         (unsigned long)xPortGetFreeHeapSize(),
         (unsigned long)xPortGetMinimumEverFreeHeapSize());
  /* creation of CANLogTask */
  CANLogTaskHandle = osThreadNew(StartTask03, NULL, &CANLogTask_attributes);
  printf("heap free=%lu min=%lu\r\n",
         (unsigned long)xPortGetFreeHeapSize(),
         (unsigned long)xPortGetMinimumEverFreeHeapSize());
  /* creation of UploadTask */
  UploadTaskHandle = osThreadNew(StartTask04, NULL, &UploadTask_attributes);
  printf("heap free=%lu min=%lu\r\n",
         (unsigned long)xPortGetFreeHeapSize(),
         (unsigned long)xPortGetMinimumEverFreeHeapSize());
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  const osEventFlagsAttr_t evt_attr = { .name = "sysEvt" };
  g_sysEvt = osEventFlagsNew(&evt_attr);  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
	    DebounceButtonAndSignal();
	    BSP_LED_Toggle(LED2);
	    osDelay(250);  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the WifiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN WifiTask */
	  WifiTask_Init();           // safe to call once; implement inside WifiApp.c

  /* Infinite loop */
  for(;;)
  {
	    WifiTask_Tick();        // keep WINC driver pumped
	    if (Wifi_HasIP()) {
	      osEventFlagsSet(g_sysEvt, EVT_HAS_IP);
	    }
	    osDelay(20);
	  }
  /* USER CODE END WifiTask */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the CANLogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN CANLogTask */
	  osEventFlagsWait(g_sysEvt, EVT_CANLOG_START, osFlagsWaitAny, osWaitForever);

	  // SD + CAN logger init (guard SD with mutex while we touch FatFs)
	  osMutexAcquire(g_sdMutex, osWaitForever);
	  DSTATUS s = SDCard_Init();
	  osMutexRelease(g_sdMutex);

	  if (s == RES_OK) {
	    CANLogger_SetLoopback(false); // optional; keep as you like
	    if (CANLogger_Init() == 0) {
	      printf("CANLogger: started\n");
	    } else {
	      printf("CANLogger: init failed\n");
	    }
	  } else {
	    printf("SD init failed, CAN logging disabled\n");
	  }
  /* Infinite loop */
  for(;;)
  {
	    // CANLogger handles its own buffering; just flush periodically.
	    osMutexAcquire(g_sdMutex, osWaitForever);
	    CANLogger_Tick();
	    osMutexRelease(g_sdMutex);
	    osDelay(50);  }
  /* USER CODE END CANLogTask */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the UploadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN UploadTask */
	  /* USER CODE BEGIN StartTask04 */
	  // 1) Wait for WiFi, 2) wait for button, 3) try upload existing CSV, 4) signal CAN logging may start.
  (void)osEventFlagsWait(g_sysEvt, EVT_HAS_IP, osFlagsWaitAny, osWaitForever);
  (void)osEventFlagsWait(g_sysEvt, EVT_BTN_PRESSED, osFlagsWaitAny, osWaitForever);

  // Make sure SD is mounted before peeking the file.
	osMutexAcquire(g_sdMutex, osWaitForever);
	DSTATUS s = SDCard_Init();
	osMutexRelease(g_sdMutex);

	if (s == RES_OK) {
	  // OPTIONAL: quick existence check to avoid needless upload
	  // Do the file I/O under mutex to avoid racing CANLogger later.
	  osMutexAcquire(g_sdMutex, osWaitForever);
	  // If your Uploader handles fopen/fread internally, just call it without mutex and remove this block.
	  int rc = Uploader_SendFileHost("0:/can_log.csv",
									 "8.tcp.us-cal-1.ngrok.io", // replace to taste
									 15868,
									 60000); // ms timeout
	  osMutexRelease(g_sdMutex);

	  printf("Upload rc=%d\n", rc);
	} else {
	  printf("Upload: SD not ready, skipping\n");
	}

	// Let the CAN logger spin up after upload window.
	osEventFlagsSet(g_sysEvt, EVT_CANLOG_START);

	    // Then park; or convert to a recurring “press-to-upload latest” loop later.

  /* Infinite loop */
  for(;;)
  {
	osDelay(1000);
  }
  /* USER CODE END UploadTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationMallocFailedHook(void) {
  printf("MALLOC FAILED free=%lu min=%lu\r\n",
         (unsigned long)xPortGetFreeHeapSize(),
         (unsigned long)xPortGetMinimumEverFreeHeapSize());
  taskDISABLE_INTERRUPTS(); for(;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  printf("STACK OVERFLOW: %s\r\n", pcTaskName);
  taskDISABLE_INTERRUPTS(); for(;;);
}

/* USER CODE END Application */

