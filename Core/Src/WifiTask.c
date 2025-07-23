// WifiTask.c
#include "WifiTask.h"
#include "WifiApp.h"      // brings up nm_bsp + m2m_wifi_init
#include "m2m_wifi.h"
#include "stm32h5xx_hal.h"

void WifiTask_Init(void)
{
    // one-time init: GPIOs, SPI, WINC driver, connect to AP
    WifiApp_InitAP();
}

void WifiTask_Tick(void)
{
    // call as often as possible to service the WINC
    m2m_wifi_handle_events(NULL);
}
