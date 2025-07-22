
// WifiApp.c
#include "main.h"
#include "WifiApp.h"
#include "bsp/include/nm_bsp_stm32h5.h"
#include <string.h>
#include <stdio.h>



/* External interrupt service routine from bus wrapper */
extern void isr(void);

/**
 * @brief Wi-Fi event callback.
 */
void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            // Station connected
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            printf("Station disconnected\r\n");
        }
        break;
    }
//    case M2M_WIFI_REQ_DHCP_CONF:
//    {
//        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
//        printf("Station connected\r\n");
//        printf("Station IP is %u.%u.%u.%u\r\n",
//               pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
//        break;
//    }
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *ip = (uint8_t*)pvMsg;
        printf("DHCP - IP address is %u.%u.%u.%u\r\n",
                ip[0], ip[1], ip[2], ip[3]);
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Initialize WINC1500 in AP mode.
 */
//void WifiApp_InitAP(void)
//{
//    tstrWifiInitParam param;
//    tstrM2MAPConfig strM2MAPConfig;
//    int8_t ret;
//
//    nm_bsp_init();
//    memset(&param, 0, sizeof(param));
//    param.pfAppWifiCb = wifi_cb;
//    ret = m2m_wifi_init(&param);
//    if (M2M_SUCCESS != ret) {
//        printf("m2m_wifi_init call error!(%d)\r\n", ret);
//        while (1) {}
//    }
//
//    memset(&strM2MAPConfig, 0, sizeof(strM2MAPConfig));
//    strcpy((char *)&strM2MAPConfig.au8SSID, MAIN_WLAN_SSID);
//    strM2MAPConfig.u8ListenChannel = MAIN_WLAN_CHANNEL;
//    strM2MAPConfig.u8SecType     = MAIN_WLAN_AUTH;
//#if USE_WEP
//    strcpy((char *)&strM2MAPConfig.au8WepKey, MAIN_WLAN_WEP_KEY);
//    strM2MAPConfig.u8KeySz    = strlen(MAIN_WLAN_WEP_KEY);
//    strM2MAPConfig.u8KeyIndx  = MAIN_WLAN_WEP_KEY_INDEX;
//#endif
//    strM2MAPConfig.au8DHCPServerIP[0] = 192;
//    strM2MAPConfig.au8DHCPServerIP[1] = 168;
//    strM2MAPConfig.au8DHCPServerIP[2] = 1;
//    strM2MAPConfig.au8DHCPServerIP[3] = 1;
//
//    ret = m2m_wifi_enable_ap(&strM2MAPConfig);
//    if (M2M_SUCCESS != ret) {
//        printf("m2m_wifi_enable_ap call error!\r\n");
//        while (1) {}
//    }
//
//    printf("AP mode started. You can connect to %s.\r\n", MAIN_WLAN_SSID);
//}
void WifiApp_InitAP(void)
{
    /* 1. Bring up HAL/BSP */
    nm_bsp_init();

    /* 2. Initialise driver and register wifi_cb() */
    tstrWifiInitParam initParam;
    memset(&initParam, 0, sizeof(initParam));
    initParam.pfAppWifiCb = wifi_cb;
    if (m2m_wifi_init(&initParam) != M2M_SUCCESS) {
        printf("WINC init failed\r\n");
        Error_Handler();
    }

    /* 3. Ask WINC to join the router */
    sint8 ret = m2m_wifi_connect(
            MAIN_WLAN_SSID,
            strlen(MAIN_WLAN_SSID),
            MAIN_WLAN_AUTH,
            (void*)MAIN_WLAN_PSK,
            MAIN_WLAN_CHANNEL);
    if (ret != M2M_SUCCESS) {
        printf("m2m_wifi_connect error %d\r\n", ret);
    } else {
        printf("Connecting to %s …\r\n", MAIN_WLAN_SSID);
    }
}

/**
 * @brief EXTI line 4 interrupt handler for WINC IRQ.
 */
void EXTI4_IRQHandler(void)
{
    /* Clear and handle interrupt */
    if (__HAL_GPIO_EXTI_GET_IT(WINC_INT_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(WINC_INT_PIN);
        HAL_GPIO_EXTI_IRQHandler(WINC_INT_PIN);
    }
}

/**
 * @brief HAL EXTI callback forwarding to WINC driver.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == WINC_INT_PIN) {
        isr();
    }
}
