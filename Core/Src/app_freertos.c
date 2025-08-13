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
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for WifiTask */
osThreadId_t WifiTaskHandle;
const osThreadAttr_t WifiTask_attributes = {
  .name = "WifiTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 4096
};
/* Definitions for CANLogTask */
osThreadId_t CANLogTaskHandle;
const osThreadAttr_t CANLogTask_attributes = {
  .name = "CANLogTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 4096
};
/* Definitions for UploadTask */
osThreadId_t UploadTaskHandle;
const osThreadAttr_t UploadTask_attributes = {
  .name = "UploadTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 2048
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of WifiTask */
  WifiTaskHandle = osThreadNew(StartTask02, NULL, &WifiTask_attributes);

  /* creation of CANLogTask */
  CANLogTaskHandle = osThreadNew(StartTask03, NULL, &CANLogTask_attributes);

  /* creation of UploadTask */
  UploadTaskHandle = osThreadNew(StartTask04, NULL, &UploadTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  printf("thr %-12s = %p  heap=%lu\r\n",
         "Default", (void*)defaultTaskHandle, (unsigned long)xPortGetFreeHeapSize());
  printf("thr %-12s = %p  heap=%lu\r\n",
         "WifiTask", (void*)WifiTaskHandle,   (unsigned long)xPortGetFreeHeapSize());
  printf("thr %-12s = %p  heap=%lu\r\n",
         "CANLogTask",(void*)CANLogTaskHandle,(unsigned long)xPortGetFreeHeapSize());
  printf("thr %-12s = %p  heap=%lu\r\n",
         "UploadTask",(void*)UploadTaskHandle,(unsigned long)xPortGetFreeHeapSize());
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/

// In app_freertos.c (Default task area)



/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
	  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

	  for (;;)
	  {
	    DebounceButtonAndSignal();      // posts EVT_BTN_PRESSED on falling edge
	    osDelay(10);
	  }


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
	  WifiTask_Init();

	  uint32_t t0 = HAL_GetTick();
	  uint32_t last_exti = 0, last_bsp = 0, last_ticks = 0;

	  for (;;)
	  {
	    WifiTask_Tick();
	    if (Wifi_HasIP()) {
	      (void)osEventFlagsSet(g_sysEvt, EVT_HAS_IP);
	    }

	    // 250 ms heartbeat (optional)
	    if (HAL_GetTick() - t0 >= 250) {
	      extern volatile uint32_t g_irq_exti_fired, g_irq_bsp_isr, g_wifi_ticks;
	      uint32_t e = g_irq_exti_fired, b = g_irq_bsp_isr, w = g_wifi_ticks;
	      printf("DBG: exti=%lu (+%lu)  bsp=%lu (+%lu)  ticks=%lu (+%lu)\r\n",
	             (unsigned long)e, (unsigned long)(e - last_exti),
	             (unsigned long)b, (unsigned long)(b - last_bsp),
	             (unsigned long)w, (unsigned long)(w - last_ticks));
	      last_exti = e; last_bsp = b; last_ticks = w;
	      t0 = HAL_GetTick();
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
	  printf("CAN: auto-start logger\r\n");

	  osMutexAcquire(g_sdMutex, osWaitForever);
	  int rc_sd = SDCard_Init();               // 0 = OK
	  osMutexRelease(g_sdMutex);
	  printf("CAN: SDCard_Init rc=%d\r\n", rc_sd);

	  if (rc_sd == 0) {
	    CANLogger_SetLoopback(false);          // real bus, not loopback
	    osMutexAcquire(g_sdMutex, osWaitForever);
	    int rc = CANLogger_Init();
	    osMutexRelease(g_sdMutex);

	    printf("CAN: CANLogger_Init rc=%d\r\n", rc);
	  } else {
	    printf("CAN: SD init failed, logging disabled\r\n");
	  }

	  for (;;)
	  {
	    osMutexAcquire(g_sdMutex, osWaitForever);
	    CANLogger_Tick();                      // drains ring + periodic f_sync
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
  /* USER CODE BEGIN UploadTask */
	  static int can_started = 0;

	  for (;;)
	   {
	     // Wait until we have BOTH Wi-Fi IP and a button press (a press before IP still counts)
	     uint32_t r = osEventFlagsWait(g_sysEvt,
	                                   EVT_HAS_IP | EVT_BTN_PRESSED,
	                                   osFlagsWaitAll | osFlagsNoClear,
	                                   osWaitForever);
	     if ((int32_t)r < 0) {
	       printf("ERR: wait IP+BTN ret=%ld\r\n", (long)(int32_t)r);
	       osDelay(50);
	       continue;
	     }

	     // Make sure SD is mounted (under mutex) before upload
	     osMutexAcquire(g_sdMutex, osWaitForever);
	     int sd_ok = (SDCard_Init() == 0);
	     osMutexRelease(g_sdMutex);
	     if (!sd_ok) {
	       printf("Upload: SD not ready\r\n");
	       osDelay(200);
	       continue;
	     }

	     // Upload (serialize all SD access under the same mutex)
	     osMutexAcquire(g_sdMutex, osWaitForever);
	     CANLogger_Suspend();

	     int rc = Uploader_SendFileHost("0:/can_log.csv",
	                                    "2.tcp.us-cal-1.ngrok.io",
	                                    11686,
	                                    60000);
	     CANLogger_Resume();
	     osMutexRelease(g_sdMutex);
	     printf("Uploader_SendFile rc=%d\r\n", rc);

	     // Consume the button bit and require release before we re-arm
	     (void)osEventFlagsClear(g_sysEvt, EVT_BTN_PRESSED);
	     while (BSP_PB_GetState(BUTTON_USER) == 0) { osDelay(10); }

	     osDelay(50);
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

