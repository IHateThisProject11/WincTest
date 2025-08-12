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
#include "main.h"
#include "cmsis_os2.h"      // make sure this include is present

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
osEventFlagsId_t g_sysEvt;  // <-- correct type
osMutexId_t      g_sdMutex; // <-- correct type
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
  .stack_size = 5120
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
  static uint32_t lastTick  = 0;
  static uint8_t  lastState = 1;   // USER button idle = 1 (released on Nucleo)
  static uint8_t  primed    = 0;

  uint8_t  s   = BSP_PB_GetState(BUTTON_USER); // 0=pressed, 1=released
  uint32_t now = osKernelGetTickCount();

  if (!primed) {  // avoid a phantom press at boot
    lastState = s;
    lastTick  = now;
    primed    = 1;
    return;
  }

  // falling edge (1->0) with 200 ms debounce
  if ((s == 0) && (lastState == 1) && ((now - lastTick) > 200)) {
    osEventFlagsSet(g_sysEvt, EVT_BTN_PRESSED);
    printf("DBG: BTN -> EVT_BTN_PRESSED\r\n");
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
	  const osMutexAttr_t      sd_mutex_attr = { .name = "sdMutex" };
	  const osEventFlagsAttr_t evt_attr      = { .name = "sysEvt" };

	  g_sdMutex = osMutexNew(&sd_mutex_attr);
	  g_sysEvt  = osEventFlagsNew(&evt_attr);         // <<< ADD THIS LINE


	  // (optional) sanity: non-NULL handle and non-zero masks
	  printf("g_sysEvt=%p HAS_IP=0x%lX BTN=0x%lX CAN=0x%lX\r\n",
	         (void*)g_sysEvt,
	         (unsigned long)EVT_HAS_IP,
	         (unsigned long)EVT_BTN_PRESSED,
	         (unsigned long)EVT_CANLOG_START);
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
	  /* create threads AFTER OS objects exist */
	  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	  printf("heap free=%lu min=%lu\r\n",
	         (unsigned long)xPortGetFreeHeapSize(),
	         (unsigned long)xPortGetMinimumEverFreeHeapSize());

	  WifiTaskHandle = osThreadNew(StartTask02, NULL, &WifiTask_attributes);
	  printf("heap free=%lu min=%lu\r\n",
	         (unsigned long)xPortGetFreeHeapSize(),
	         (unsigned long)xPortGetMinimumEverFreeHeapSize());

	  CANLogTaskHandle = osThreadNew(StartTask03, NULL, &CANLogTask_attributes);
	  printf("heap free=%lu min=%lu\r\n",
	         (unsigned long)xPortGetFreeHeapSize(),
	         (unsigned long)xPortGetMinimumEverFreeHeapSize());

	  UploadTaskHandle = osThreadNew(StartTask04, NULL, &UploadTask_attributes);
	  printf("heap free=%lu min=%lu\r\n",
	         (unsigned long)xPortGetFreeHeapSize(),
	         (unsigned long)xPortGetMinimumEverFreeHeapSize());
	}
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */


/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/

// In app_freertos.c (Default task area)



/* USER CODE END Header_StartDefaultTask */
// In app_freertos.c (Default task area)

void StartDefaultTask(void *argument)
{
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  for (;;) {
    DebounceButtonAndSignal();
    osDelay(10);
  }
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
  static uint32_t t0 = 0;
  static uint32_t last_exti = 0, last_bsp = 0, last_ticks = 0;

  WifiTask_Init();

  for (;;) {
    WifiTask_Tick();
    if (Wifi_HasIP()) {
      osEventFlagsSet(g_sysEvt, EVT_HAS_IP);
    }

    if (HAL_GetTick() - t0 >= 250) {
      uint32_t e = g_irq_exti_fired;
      uint32_t b = g_irq_bsp_isr;
      uint32_t w = g_wifi_ticks;
      printf("DBG: exti=%lu (+%lu)  bsp=%lu (+%lu)  ticks=%lu (+%lu)\r\n",
             (unsigned long)e, (unsigned long)(e - last_exti),
             (unsigned long)b, (unsigned long)(b - last_bsp),
             (unsigned long)w, (unsigned long)(w - last_ticks));
      last_exti = e; last_bsp = b; last_ticks = w;
      t0 = HAL_GetTick();
    }

    osDelay(20);
  }
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

	osMutexAcquire(g_sdMutex, osWaitForever);
	int rc = SDCard_Init();    // 0 = OK
	osMutexRelease(g_sdMutex);

	if (rc == 0) {
	  CANLogger_SetLoopback(false);
	  if (CANLogger_Init() == 0) {
	    printf("CANLogger: started\n");
	  } else {
	    printf("CANLogger: init failed\n");
	  }
	} else {
	  printf("SD init failed, CAN logging disabled (rc=%d)\n", rc);
	}

	for (;;) {
	  osMutexAcquire(g_sdMutex, osWaitForever);
	  CANLogger_Tick();
	  osMutexRelease(g_sdMutex);
	  osDelay(50);
	}

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
  // One-shot throttle for CAN start (optional but tidy)
  static int can_started = 0;

  for (;;) {
    // 1) Wait for IP (if already set, this returns immediately).
    uint32_t r = osEventFlagsWait(g_sysEvt, EVT_HAS_IP, osFlagsWaitAny, osWaitForever);
    if ((int32_t)r < 0) {                   // <-- treat error as "don't proceed"
      printf("ERR: wait HAS_IP ret=%ld\r\n", (long)(int32_t)r);
      osDelay(50);
      continue;
    }

    // 2) Require a *fresh* press each time.
    (void)osEventFlagsClear(g_sysEvt, EVT_BTN_PRESSED); // consume any stale bit

    r = osEventFlagsWait(g_sysEvt, EVT_BTN_PRESSED, osFlagsWaitAny, osWaitForever);
    if ((int32_t)r < 0) {
      printf("ERR: wait BTN ret=%ld\r\n", (long)(int32_t)r);
      osDelay(50);
      continue;
    }

    // Optional: require release before re-arming so a held key can't re-trigger.
    while (BSP_PB_GetState(BUTTON_USER) == 0) {
      osDelay(10);
    }

    // 3) Ensure SD is ready (under mutex) *once per press*.
    osMutexAcquire(g_sdMutex, osWaitForever);
    int sd_ok = (SDCard_Init() == 0);  // your SDCard_Init returns 0 on success
    osMutexRelease(g_sdMutex);

    if (!sd_ok) {
      printf("Upload: SD not ready, skipping\r\n");
      osDelay(200);
      continue;                        // wait for next press
    }

    // 4) Upload the file (protected by the same SD mutex).
    osMutexAcquire(g_sdMutex, osWaitForever);
    int rc = Uploader_SendFileHost("0:/can_log.csv",
                                   "8.tcp.us-cal-1.ngrok.io",
                                   15868,
                                   60000);
    osMutexRelease(g_sdMutex);
    printf("Uploader_SendFile rc=%d\r\n", rc);

    // 5) Allow CAN logging to start the first time only (harmless if repeated).
    if (!can_started) {
      (void)osEventFlagsSet(g_sysEvt, EVT_CANLOG_START);
      can_started = 1;
    }

    // 6) Clear BTN bit again to force a *new* press next loop.
    (void)osEventFlagsClear(g_sysEvt, EVT_BTN_PRESSED);

    osDelay(50);  // small guard delay
  }
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

