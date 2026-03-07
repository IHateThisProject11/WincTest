
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



void WifiApp_InitAP(void)
{
//    nm_bsp_init();
//
//    tstrWifiInitParam initParam;
//    memset(&initParam, 0, sizeof(initParam));
//    initParam.pfAppWifiCb = wifi_cb;
//    if (m2m_wifi_init(&initParam) != M2M_SUCCESS) {
//        M2M_ERR("WINC init failed\r\n");
//        Error_Handler();
//    }

    sint8 ret = m2m_wifi_connect(
            MAIN_WLAN_SSID,
            strlen(MAIN_WLAN_SSID),
            MAIN_WLAN_AUTH,
            (void*)MAIN_WLAN_PSK,
            MAIN_WLAN_CHANNEL);
    M2M_INFO("m2m_wifi_connect rc=%d\r\n", ret);

    if (ret != M2M_SUCCESS) {
        M2M_ERR("m2m_wifi_connect error %d\r\n", ret);
    } else {
        M2M_INFO("Connecting to %s\r\n", MAIN_WLAN_SSID);
    }
}


//static void raw_winc_test(void)
//{
//    extern SPI_HandleTypeDef hspi1;
//    HAL_StatusTypeDef st;
//    uint8_t tx[16], rx[16];
//
//    M2M_INFO("\r\n========= RAW WINC SPI TEST =========\r\n");
//
//    /* 1. Full power cycle */
//    M2M_INFO("Step 1: Power cycle WINC\r\n");
//    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(CHIP_EN_WINC_GPIO_Port, CHIP_EN_WINC_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(RESET_WINC_GPIO_Port, RESET_WINC_Pin, GPIO_PIN_RESET);
//    HAL_Delay(500);
//    HAL_GPIO_WritePin(CHIP_EN_WINC_GPIO_Port, CHIP_EN_WINC_Pin, GPIO_PIN_SET);
//    HAL_Delay(200);
//    HAL_GPIO_WritePin(RESET_WINC_GPIO_Port, RESET_WINC_Pin, GPIO_PIN_SET);
//    M2M_INFO("  Waiting 2s for boot...\r\n");
//    HAL_Delay(2000);
//    M2M_INFO("  IRQ=%d  MISO=%d\r\n",
//        HAL_GPIO_ReadPin(IRQ_WINC_PIN_GPIO_Port, IRQ_WINC_PIN_Pin),
//        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
//
//    /* 2. Fresh SPI init for EACH test (avoids poisoned state) */
//    /* Helper macro to do clean init */
//    #define REINIT_SPI(swap, cpol, cpha) do { \
//        HAL_SPI_DeInit(&hspi1); \
//        hspi1.Instance               = SPI1; \
//        hspi1.Init.Mode              = SPI_MODE_MASTER; \
//        hspi1.Init.Direction         = SPI_DIRECTION_2LINES; \
//        hspi1.Init.DataSize          = SPI_DATASIZE_8BIT; \
//        hspi1.Init.CLKPolarity       = (cpol); \
//        hspi1.Init.CLKPhase          = (cpha); \
//        hspi1.Init.NSS               = SPI_NSS_SOFT; \
//        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; \
//        hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB; \
//        hspi1.Init.TIMode            = SPI_TIMODE_DISABLE; \
//        hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; \
//        hspi1.Init.CRCPolynomial     = 0x7; \
//        hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE; \
//        hspi1.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW; \
//        hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA; \
//        hspi1.Init.MasterSSIdleness          = SPI_MASTER_SS_IDLENESS_00CYCLE; \
//        hspi1.Init.MasterInterDataIdleness   = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE; \
//        hspi1.Init.MasterReceiverAutoSusp    = SPI_MASTER_RX_AUTOSUSP_DISABLE; \
//        hspi1.Init.MasterKeepIOState         = SPI_MASTER_KEEP_IO_STATE_DISABLE; \
//        hspi1.Init.IOSwap                    = (swap); \
//        hspi1.Init.ReadyMasterManagement     = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY; \
//        hspi1.Init.ReadyPolarity             = SPI_RDY_POLARITY_HIGH; \
//        HAL_SPI_Init(&hspi1); \
//    } while(0)
//
//    /* ---- Test A: Normal Mode 0, no swap ---- */
//    M2M_INFO("Test A: Mode 0, normal pins\r\n");
//    REINIT_SPI(SPI_IO_SWAP_DISABLE, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
//
//    memset(rx, 0xDE, 16);
//    tx[0]=0xCA; tx[1]=0x00; tx[2]=0x10; tx[3]=0x00; tx[4]=0xCA;
//    for(int i=5; i<16; i++) tx[i]=0x00;
//
//    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1);
//    st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 16, 500);
//    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_SET);
//    M2M_INFO("  st=%d RX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//        (int)st, rx[0],rx[1],rx[2],rx[3],rx[4],rx[5],rx[6],rx[7]);
//    M2M_INFO("           %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//        rx[8],rx[9],rx[10],rx[11],rx[12],rx[13],rx[14],rx[15]);
//    HAL_Delay(50);
//
//    /* ---- Test B: IO Swap (MOSI/MISO swapped) ---- */
//    M2M_INFO("Test B: Mode 0, IO swap\r\n");
//    REINIT_SPI(SPI_IO_SWAP_ENABLE, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
//
//    memset(rx, 0xDE, 16);
//    tx[0]=0xCA; tx[1]=0x00; tx[2]=0x10; tx[3]=0x00; tx[4]=0xCA;
//    for(int i=5; i<16; i++) tx[i]=0x00;
//
//    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_RESET);
//    HAL_Delay(1);
//    st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 16, 500);
//    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_SET);
//    M2M_INFO("  st=%d RX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//        (int)st, rx[0],rx[1],rx[2],rx[3],rx[4],rx[5],rx[6],rx[7]);
//    M2M_INFO("           %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//        rx[8],rx[9],rx[10],rx[11],rx[12],rx[13],rx[14],rx[15]);
//    HAL_Delay(50);
//
//    /* ---- Test C: CS HIGH baseline (no WINC) ---- */
//    M2M_INFO("Test C: CS HIGH (WINC deselected, expect FF)\r\n");
//    REINIT_SPI(SPI_IO_SWAP_DISABLE, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
//
//    memset(rx, 0xDE, 4);
//    memset(tx, 0xAA, 4);
//    HAL_GPIO_WritePin(CS_WINC_GPIO_Port, CS_WINC_Pin, GPIO_PIN_SET);
//    st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 500);
//    M2M_INFO("  st=%d RX: %02X %02X %02X %02X\r\n",
//        (int)st, rx[0],rx[1],rx[2],rx[3]);
//
//    /* ---- Test D: SPI register dump ---- */
//    M2M_INFO("Test D: SPI1 registers\r\n");
//    M2M_INFO("  CR1=0x%08lX  CFG1=0x%08lX\r\n", SPI1->CR1, SPI1->CFG1);
//    M2M_INFO("  CFG2=0x%08lX  SR=0x%08lX\r\n", SPI1->CFG2, SPI1->SR);
//
//    M2M_INFO("========= END TEST =========\r\n");
//    while(1) { HAL_Delay(1000); }
//}

//void WifiApp_InitAP(void)
//{
//    raw_winc_test();  /* replace normal init with test */
//}

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
