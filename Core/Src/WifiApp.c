
// WifiApp.c
#include "main.h"
#include "WifiApp.h"
#include "bsp/include/nm_bsp_stm32h5.h"
#include <string.h>
#include <stdio.h>
#include "m2m_wifi.h"
#include "nm_bsp.h"
#include "socket.h"
#include "nm_bsp.h"
/* External interrupt service routine from bus wrapper */
extern void isr(void);
extern void nm_bsp_call_isr(void); /* declared in the BSP */
static volatile bool s_wifi_has_ip = false;


/* --- debug counters --- */
volatile uint32_t g_irq_exti_fired = 0;    // HAL EXTI callback hit
volatile uint32_t g_irq_bsp_isr    = 0;    // nm_bsp_call_isr invoked
volatile uint32_t g_wifi_ticks     = 0;    // WifiTask_Tick calls

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
            s_wifi_has_ip = false;
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
        printf("DHCP - IP address is %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
        s_wifi_has_ip = true;
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
static void winc_hardware_diag(void)
{
    M2M_INFO("\r\n=== WINC HARDWARE DIAGNOSTIC ===\r\n");

    M2M_INFO("BEFORE power-up:\r\n");
    M2M_INFO("  CHIP_EN (PB1) = %d\r\n",
           HAL_GPIO_ReadPin(CHIP_EN_WINC_GPIO_Port, CHIP_EN_WINC_Pin));
    M2M_INFO("  RESET   (PB0) = %d\r\n",
           HAL_GPIO_ReadPin(RESET_WINC_GPIO_Port, RESET_WINC_Pin));
    M2M_INFO("  CS      (PC5) = %d\r\n",
           HAL_GPIO_ReadPin(CS_WINC_GPIO_Port, CS_WINC_Pin));
    M2M_INFO("  IRQ     (PC4) = %d\r\n",
           HAL_GPIO_ReadPin(IRQ_WINC_PIN_GPIO_Port, IRQ_WINC_PIN_Pin));
    M2M_INFO("  MISO    (PA6) = %d\r\n",
           HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));

    M2M_INFO("Powering WINC: CHIP_EN=0, RESET=0\r\n");
    HAL_GPIO_WritePin(CHIP_EN_WINC_GPIO_Port, CHIP_EN_WINC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RESET_WINC_GPIO_Port, RESET_WINC_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    M2M_INFO("Setting CHIP_EN=1\r\n");
    HAL_GPIO_WritePin(CHIP_EN_WINC_GPIO_Port, CHIP_EN_WINC_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    M2M_INFO("Setting RESET=1\r\n");
    HAL_GPIO_WritePin(RESET_WINC_GPIO_Port, RESET_WINC_Pin, GPIO_PIN_SET);
    HAL_Delay(500);

    M2M_INFO("AFTER power-up:\r\n");
    M2M_INFO("  IRQ     (PC4) = %d\r\n",
           HAL_GPIO_ReadPin(IRQ_WINC_PIN_GPIO_Port, IRQ_WINC_PIN_Pin));
    M2M_INFO("  MISO    (PA6) = %d\r\n",
           HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
    M2M_INFO("=== END PRE-INIT DIAGNOSTIC ===\r\n\r\n");
}

static void winc_spi_diag_post_init(void)
{
    extern SPI_HandleTypeDef hspi1;

    M2M_INFO("\r\n=== POST-INIT SPI DIAGNOSTIC ===\r\n");

    // Test 1: CS HIGH (WINC deselected). MISO floats with pull-up.
    // Should read FF FF FF FF.
    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    uint8_t tx1[4] = {0xAA, 0x55, 0xAA, 0x55};
    uint8_t rx1[4] = {0xDE, 0xDE, 0xDE, 0xDE};
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx1, rx1, 4, 100);
    M2M_INFO("CS=HIGH: status=%d RX: %02X %02X %02X %02X (expect FF)\r\n",
           st, rx1[0], rx1[1], rx1[2], rx1[3]);

    // Test 2: CS LOW (WINC selected). Send CMD_SINGLE_READ for chip ID.
    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    uint8_t tx2[8] = {0xCA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx2[8] = {0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE};
    st = HAL_SPI_TransmitReceive(&hspi1, tx2, rx2, 8, 100);
    M2M_INFO("CS=LOW:  status=%d RX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           st, rx2[0], rx2[1], rx2[2], rx2[3],
           rx2[4], rx2[5], rx2[6], rx2[7]);
    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_SET);

    // Test 3: Read SPI registers directly to verify config
    M2M_INFO("SPI1->CR1  = 0x%08lX\r\n", SPI1->CR1);
    M2M_INFO("SPI1->CFG1 = 0x%08lX\r\n", SPI1->CFG1);
    M2M_INFO("SPI1->CFG2 = 0x%08lX\r\n", SPI1->CFG2);
    M2M_INFO("SPI1->SR   = 0x%08lX\r\n", SPI1->SR);

    M2M_INFO("=== END POST-INIT DIAGNOSTIC ===\r\n\r\n");
}


void WifiApp_InitAP(void)
{
    winc_hardware_diag();
    nm_bsp_init();

    // nm_bus_init runs inside m2m_wifi_init, but let's test SPI
    // right after BSP init, which does reset + power-up.
    // We need SPI configured first, so call nm_bus_init manually:
    extern sint8 nm_bus_init(void *);
    nm_bus_init(NULL);

    winc_spi_diag_post_init();  // <-- NEW: test SPI after init

    // Now proceed with normal WINC init
    tstrWifiInitParam initParam;
    memset(&initParam, 0, sizeof(initParam));
    initParam.pfAppWifiCb = wifi_cb;
    if (m2m_wifi_init(&initParam) != M2M_SUCCESS) {
        M2M_ERR("WINC init failed\r\n");
        Error_Handler();
    }

    sint8 ret = m2m_wifi_connect(
            MAIN_WLAN_SSID,
            strlen(MAIN_WLAN_SSID),
            MAIN_WLAN_AUTH,
            (void*)MAIN_WLAN_PSK,
            MAIN_WLAN_CHANNEL);
    M2M_INFO("m2m_wifi_connect rc=%d\r\n", ret);

    if (ret != M2M_SUCCESS) {
        printf("m2m_wifi_connect error %d\r\n", ret);
    } else {
        printf("Connecting to %s …\r\n", MAIN_WLAN_SSID);
    }
}

/**
// * @brief EXTI line 4 interrupt handler for WINC IRQ.
// */
//void EXTI4_IRQHandler(void)
//{
//    /* Clear and handle interrupt */
//    if (__HAL_GPIO_EXTI_GET_IT(WINC_INT_PIN) != RESET) {
//        __HAL_GPIO_EXTI_CLEAR_IT(WINC_INT_PIN);
//        HAL_GPIO_EXTI_IRQHandler(WINC_INT_PIN);
//    }
//}

/**
 * @brief HAL EXTI callback forwarding to WINC driver.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IRQ_WINC_PIN_Pin) {
        g_irq_exti_fired++;      // count callback entries (no prints here)
        nm_bsp_call_isr();       // invoke the WINC driver’s registered ISR
        g_irq_bsp_isr++;         // count BSP trampoline calls
    }
}


bool Wifi_HasIP(void)
{
    return s_wifi_has_ip;
}
