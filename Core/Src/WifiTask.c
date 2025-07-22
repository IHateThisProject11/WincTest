#include "WifiTask.h"
#include "WifiApp.h"        // your init functions & callback registration
#include "m2m_wifi.h"       // WINC1500 driver
#include <stdio.h>          // for printf()

void WifiTask(void *argument)
{
    // 1) Initialize the WINC1500 driver in AP or STA mode:
    //    this will call m2m_wifi_init() under the hood.
    WifiApp_InitAP();    // or WifiApp_InitSTA()


}
