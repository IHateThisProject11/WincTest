#include "CANLogger.h"
#include "ff.h"                        // FatFS
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"
#include <stdio.h>
#include <string.h>

/* extern from CubeMX */
extern FDCAN_HandleTypeDef hfdcan1;

/* ---------- Config ---------- */
#ifndef CANCSV_PATH
#define CANCSV_PATH        "0:/can_log.csv"
#endif

#ifndef CANCSV_FLUSH_EVERY
#define CANCSV_FLUSH_EVERY  50u   /* flush file every N frames */
#endif

/* ---------- State ---------- */
static FIL s_fil;
static bool s_file_open   = false;
static bool s_loopback    = false;
static uint32_t s_rx_count = 0;

/* Map FDCAN DLC macro to byte length (classic 0..8) */
static uint8_t _dlc_bytes(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 8;
    }
}

void CANLogger_SetLoopback(bool enable) { s_loopback = enable; }
bool CANLogger_Ready(void)              { return s_file_open; }

/* Write header if file just created (size == 0) */
static void _maybe_write_header(void) {
    if (f_size(&s_fil) == 0) {
        const char *hdr = "ms,id_type,id_hex,dlc,d0,d1,d2,d3,d4,d5,d6,d7\r\n";
        UINT bw=0; f_write(&s_fil, hdr, (UINT)strlen(hdr), &bw);
        f_sync(&s_fil);
    }
}

/* HAL FDCAN callback: RX FIFO0 new message */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t itflags)
{
    if (hfdcan != &hfdcan1) return;
    if ((itflags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

    FDCAN_RxHeaderTypeDef rxh;
    uint8_t data[8] = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, data) != HAL_OK) {
        return;
    }

    uint32_t t = HAL_GetTick();
    char line[96];
    const char *idtype = (rxh.IdType == FDCAN_STANDARD_ID) ? "std" : "ext";
    uint8_t len = _dlc_bytes(rxh.DataLength);

    int n = snprintf(line, sizeof(line),
                     "%lu,%s,%03lX,%u,"
                     "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                     (unsigned long)t, idtype,
                     (unsigned long)rxh.Identifier,
                     (unsigned)len,
                     len>0?data[0]:0, len>1?data[1]:0, len>2?data[2]:0, len>3?data[3]:0,
                     len>4?data[4]:0, len>5?data[5]:0, len>6?data[6]:0, len>7?data[7]:0);

    if (s_file_open && n > 0) {
        UINT bw=0; f_write(&s_fil, line, (UINT)n, &bw);
        s_rx_count++;
        if (s_rx_count % CANCSV_FLUSH_EVERY == 0) {
            f_sync(&s_fil);
        }
    }
}

static int _fdcan_setup(void)
{
    if (s_loopback) {
        /* If you want loopback for bench test, do this BEFORE HAL_FDCAN_Init in fdcan.c
           or re-init here if needed. If Cube already initialized, skip this. */
        /* Example if re-initting here:
         * hfdcan1.Instance = FDCAN1;
         * hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
         */
    }

    /* Accept all 11-bit IDs to RX FIFO0 */
    FDCAN_FilterTypeDef flt = {0};
    flt.IdType       = FDCAN_STANDARD_ID;
    flt.FilterIndex  = 0;
    flt.FilterType   = FDCAN_FILTER_RANGE_NO_EIDM;
    flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    flt.FilterID1    = 0x000;
    flt.FilterID2    = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &flt) != HAL_OK) return -20;

    /* (Optional) also accept all 29-bit IDs */
    flt.IdType       = FDCAN_EXTENDED_ID;
    flt.FilterIndex  = 1;
    flt.FilterID1    = 0x00000000;
    flt.FilterID2    = 0x1FFFFFFF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &flt) != HAL_OK) return -21;

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        return -22;

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return -23;

    return 0;
}

int CANLogger_Init(void)
{
    /* Open/append the CSV file */
    FRESULT fr = f_open(&s_fil, CANCSV_PATH, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        printf("CANCSV: f_open('%s') failed rc=%u\r\n", CANCSV_PATH, (unsigned)fr);
        return -1;
    }
    s_file_open = true;
    _maybe_write_header();

    /* Configure filters and start FDCAN */
    int rc = _fdcan_setup();
    if (rc != 0) {
        printf("CANCSV: FDCAN setup failed rc=%d\r\n", rc);
        f_close(&s_fil);
        s_file_open = false;
        return rc;
    }
    printf("CANCSV: logging to %s\r\n", CANCSV_PATH);
    return 0;
}

void CANLogger_Tick(void)
{
    /* Periodic flush (in case of low traffic) */
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (s_file_open && (now - last) >= 1000U) {
        f_sync(&s_fil);
        last = now;
    }
}
