// Core/Src/WifiTask.c

#include "WifiTask.h"
#include "WifiApp.h"        // WifiApp_InitAP()
#include "m2m_wifi.h"       // WINC1500 driver API
#include "stm32h5xx_hal.h"  // HAL_GetTick()
#include <stdio.h>

/**
 * @brief  “Bare-metal” replacement for the old FreeRTOS task.
 *         Call this from main() (or spawn as a thread): it never returns.
 */
void WifiTask(void *argument)
{
    /* 1) One-time init: pull in nm_bsp, init driver and connect */
    WifiApp_InitAP();    /* brings up SPI + IRQ handlers + calls m2m_wifi_init() */

    /* 2) Service the WINC in a tight loop */
    uint32_t lastTick = HAL_GetTick();  // ms

    for (;;)
    {
        /* This MUST be called periodically and independently of any other condition */
        m2m_wifi_handle_events(NULL);

        /* 3) Optional 10 ms “heartbeat” for your own periodic work */
        if ((HAL_GetTick() - lastTick) >= 10U)
        {
            lastTick += 10U;
            // e.g. check link status: if (m2m_wifi_get_state() == M2M_WIFI_CONNECTED) { … }
        }
    }
}
