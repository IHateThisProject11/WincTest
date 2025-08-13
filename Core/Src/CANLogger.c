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

/* Human-readable mode name for a one-line init print */
static const char* _mode_str(uint32_t m) {
    switch (m) {
        case FDCAN_MODE_NORMAL:               return "NORMAL";
        case FDCAN_MODE_BUS_MONITORING:       return "BUS_MONITOR";
        case FDCAN_MODE_RESTRICTED_OPERATION: return "RESTRICTED";
        case FDCAN_MODE_INTERNAL_LOOPBACK:    return "INT_LOOP";
        case FDCAN_MODE_EXTERNAL_LOOPBACK:    return "EXT_LOOP";
        default: return "UNKNOWN";
    }
}


/* ---------- State ---------- */
static FIL s_fil;
static bool s_file_open   = false;
static bool s_loopback    = false;
static uint32_t s_rx_count = 0;

/* ---------- Debug toggles ---------- */
#ifndef CANDBG_PRINT_FIRST_N
#define CANDBG_PRINT_FIRST_N   10u   /* echo first N frames to UART */
#endif
#ifndef CANDBG_HEARTBEAT_MS
#define CANDBG_HEARTBEAT_MS  1000u   /* periodic counters print */
#endif
#ifndef CANLOG_USE_ISR_WRITES
#define CANLOG_USE_ISR_WRITES  0     /* 0 = queue in ISR, write in Tick */
#endif

/* ---------- Counters ---------- */
static volatile uint32_t s_irq_rx        = 0;
static volatile uint32_t s_fifo_full     = 0;
static volatile uint32_t s_fifo_lost     = 0;
static volatile uint32_t s_write_err     = 0;

/* ---------- Optional ring for CSV lines ---------- */
#if (CANLOG_USE_ISR_WRITES == 0)
#define QCAP 64
typedef struct { uint16_t n; char line[96]; } csv_t;
static csv_t s_q[QCAP];
static volatile uint16_t s_qh = 0, s_qt = 0;
static inline int _qpush(const char* s, uint16_t n){
    uint16_t nh = (uint16_t)((s_qh + 1u) % QCAP);
    if (nh == s_qt) { s_fifo_lost++; return -1; }
    s_q[s_qh].n = n; memcpy(s_q[s_qh].line, s, n);
    s_qh = nh; return 0;
}
static inline int _qpop(csv_t* out){
    if (s_qt == s_qh) return -1;
    *out = s_q[s_qt];
    s_qt = (uint16_t)((s_qt + 1u) % QCAP);
    return 0;
}
#endif

int CANLogger_Suspend(void) {
    if (s_file_open) { f_sync(&s_fil); f_close(&s_fil); s_file_open = false; }
    return 0;
}

int CANLogger_Resume(void) {
    if (!s_file_open) {
        FRESULT fr = f_open(&s_fil, CANCSV_PATH, FA_OPEN_ALWAYS | FA_WRITE);
        if (fr != FR_OK) return -1;
        f_lseek(&s_fil, f_size(&s_fil));
        s_file_open = true;
    }
    return 0;
}

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

    if (itflags & FDCAN_IT_RX_FIFO0_FULL) {
        s_fifo_full++;
    }
    if ((itflags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

    s_irq_rx++;

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
    if (n <= 0) return;

#if CANDBG_PRINT_FIRST_N
    static uint32_t left = CANDBG_PRINT_FIRST_N;
    if (left) {
        printf("CAN rx t=%lu %s 0x%lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
               (unsigned long)t, idtype, (unsigned long)rxh.Identifier, (unsigned)len,
               data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
        left--;
    }
#endif

#if (CANLOG_USE_ISR_WRITES)
    if (s_file_open) {
        UINT bw=0;
        if (f_write(&s_fil, line, (UINT)n, &bw) != FR_OK || bw != (UINT)n) s_write_err++;
        s_rx_count++;
        if (s_rx_count % CANCSV_FLUSH_EVERY == 0) { f_sync(&s_fil); }
    }
#else
    (void)_qpush(line, (uint16_t)n);
#endif
}


void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan != &hfdcan1) return;

    uint32_t ir  = hfdcan->Instance->IR;
    uint32_t ecr = hfdcan->Instance->ECR;

    /* TEC/REC/LEC are super useful while sniffing a live bus */
    printf("FDCAN ERR: IR=0x%08lX ECR=0x%08lX TEC=%lu REC=%lu LEC=%lu\r\n",
           (unsigned long)ir, (unsigned long)ecr,
           (unsigned long)((ecr >> 16) & 0xFF),   /* TEC */
           (unsigned long)( ecr        & 0x7F),   /* REC */
           (unsigned long)((ecr >> 8)  & 0x7));   /* LEC */

    /* Clear the error flags that are defined on STM32H5 */
    uint32_t clr = 0;
    #ifdef FDCAN_FLAG_ERROR_WARNING
    clr |= FDCAN_FLAG_ERROR_WARNING;
    #endif
    #ifdef FDCAN_FLAG_ERROR_PASSIVE
    clr |= FDCAN_FLAG_ERROR_PASSIVE;
    #endif
    #ifdef FDCAN_FLAG_BUS_OFF
    clr |= FDCAN_FLAG_BUS_OFF;
    #endif

    if (clr) {
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, clr);
    }
}


static int _fdcan_setup(void)
{
    /* Accept all 11-bit IDs to RX FIFO0 */
    FDCAN_FilterTypeDef flt = {0};
    flt.IdType       = FDCAN_STANDARD_ID;
    flt.FilterIndex  = 0;
    flt.FilterType   = FDCAN_FILTER_RANGE_NO_EIDM;
    flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    flt.FilterID1    = 0x000;
    flt.FilterID2    = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &flt) != HAL_OK) return -20;

    /* Also accept all 29-bit IDs */
    flt.IdType       = FDCAN_EXTENDED_ID;
    flt.FilterIndex  = 1;
    flt.FilterID1    = 0x00000000;
    flt.FilterID2    = 0x1FFFFFFF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &flt) != HAL_OK) return -21;

    /* RX notifications: new msg, full, lost */
    uint32_t its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                   FDCAN_IT_RX_FIFO0_FULL |
                   FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, its, 0) != HAL_OK) return -22;

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return -23;

    /* One-line timing + mode banner */
    printf("FDCAN: mode=%s presc=%lu seg1=%lu seg2=%lu sjw=%lu\r\n",
           _mode_str(hfdcan1.Init.Mode),
           (unsigned long)hfdcan1.Init.NominalPrescaler,
           (unsigned long)hfdcan1.Init.NominalTimeSeg1,
           (unsigned long)hfdcan1.Init.NominalTimeSeg2,
           (unsigned long)hfdcan1.Init.NominalSyncJumpWidth);
    return 0;
}



int CANLogger_Init(void)
{
    /* Open/append the CSV file */
//    FRESULT fr = f_open(&s_fil, CANCSV_PATH, FA_OPEN_APPEND | FA_WRITE);
//    if (fr != FR_OK) {
//        printf("CANCSV: f_open('%s') failed rc=%u\r\n", CANCSV_PATH, (unsigned)fr);
//        return -1;
//    }
	// was: f_open(&s_fil, CANCSV_PATH, FA_OPEN_APPEND | FA_WRITE);
	FRESULT fr = f_open(&s_fil, CANCSV_PATH, FA_OPEN_ALWAYS | FA_WRITE);
	if (fr == FR_OK) {
	    f_lseek(&s_fil, f_size(&s_fil));  // move to end to append
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
    uint32_t now = HAL_GetTick();

#if (CANLOG_USE_ISR_WRITES == 0)
    if (s_file_open) {
        csv_t item;
        while (_qpop(&item) == 0) {
        	UINT bw = 0;
        	FRESULT fr = f_write(&s_fil, item.line, (UINT)item.n, &bw);
        	if (fr != FR_OK || bw != item.n) {
        	    // quick retry after a sync; keeps us resilient to sporadic card busy
        	    f_sync(&s_fil);
        	    bw = 0;
        	    fr = f_write(&s_fil, item.line, (UINT)item.n, &bw);
        	}
        	if (fr != FR_OK || bw != item.n) {
        	    s_write_err++;
        	    break; // still bad; back off this tick
        	}

            s_rx_count++;
        }
        /* periodic flush */
        static uint32_t last = 0;
        if ((now - last) >= 250U) { f_sync(&s_fil); last = now; }
    }
#else
    /* original “flush once per second” path if writing in ISR */
    static uint32_t last = 0;
    if (s_file_open && (now - last) >= 1000U) { f_sync(&s_fil); last = now; }
#endif

#if CANDBG_HEARTBEAT_MS
    static uint32_t dbg_last = 0;
    if ((now - dbg_last) >= CANDBG_HEARTBEAT_MS) {
        printf("CANDBG: irq=%lu rx=%lu q=%u/%u full=%lu lost=%lu werr=%lu file=%d\r\n",
               (unsigned long)s_irq_rx, (unsigned long)s_rx_count,
#if (CANLOG_USE_ISR_WRITES == 0)
               (unsigned)((s_qh + QCAP - s_qt) % QCAP), (unsigned)QCAP,
#else
               0u, 0u,
#endif
               (unsigned long)s_fifo_full, (unsigned long)s_fifo_lost,
               (unsigned long)s_write_err, s_file_open ? 1 : 0);
        dbg_last = now;
    }
#endif
}

