#ifndef WIFIAPP_H
#define WIFIAPP_H

#ifdef __cplusplus
extern "C" {
#endif

/* STM32H5 HAL + board support */
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"    // change to your exact board header
#include <stdbool.h>

/* Standard lib for printf() */
#include <stdio.h>

/* WINC1500 types & APIs */
#include "driver/include/m2m_types.h"
//#include "driver/include/m2m_wifi.h"
#include "m2m_wifi.h"

bool Wifi_HasIP(void);

/* === IRQ pin alias === */
#define IRQ_WINC_Pin    IRQ_WINC_PIN_Pin   /* CubeMX name */
#define WINC_INT_PIN    IRQ_WINC_Pin



/* === Wi-Fi network settings (station mode) === */
#define USE_WEP              0                /* leave at 0 for WPA2-PSK */
#define MAIN_WLAN_SSID    "PVans 2.4"        /* e.g. your home router or PC hotspot name */
#define MAIN_WLAN_AUTH    M2M_WIFI_SEC_WPA_PSK       /* WPA2-PSK; use M2M_WIFI_SEC_OPEN if no passphrase */
#define MAIN_WLAN_PSK     "8QS4iO2x"    /* your Wi-Fi password (8–63 chars) */
#define MAIN_WLAN_CHANNEL M2M_WIFI_CH_ALL            /* scan all channels */


/* === Public API === */
/**
 * @brief Initialize the WINC1500 in Access-Point mode.
 */
void WifiApp_InitAP(void);


/**
 * @brief Wi-Fi event callback (registered with m2m_wifi_init).
 */
void wifi_cb(uint8_t u8MsgType, void *pvMsg);

/**
 * @brief EXTI line IRQ for WINC1500 interrupt pin.
 */
void EXTI4_IRQHandler(void);

/**
 * @brief Forward HAL EXTI callbacks to the WINC ISR.
 */

  /* your PC13/button EXTI handling if you still use it */

#ifdef __cplusplus
}
#endif

#endif /* WIFIAPP_H */
