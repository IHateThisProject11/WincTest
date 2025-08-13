#include "SDCard.h"
#include "main.h"    /* brings in hspi3, SDCARD_CS_Pin/Port, and retargeted printf */
#include <string.h>  /* for strlen */
#include <stdio.h>   /* for printf */
#include "ff.h"
#include "stm32h5xx_hal.h"
#include "diskio.h"


// file-scope (make sure these exist only once in SDCard.c)
static FATFS SDFatFS;                  /* File system object for logical drive 0 */
extern SPI_HandleTypeDef hspi3;
static int s_sd_mounted = 0;           /* 0 = not mounted, 1 = mounted */

/* Pulse 8 clocks with CS high so the card can release DO after writes;
   also issue a CTRL_SYNC to wait out the internal busy time. */
// SDCard.c  (keep the same signature; replace body)
void SDCard_Quiesce(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    uint8_t ff[4] = {0xFF,0xFF,0xFF,0xFF};
    extern SPI_HandleTypeDef hspi3;
    (void)HAL_SPI_Transmit(&hspi3, ff, sizeof(ff), 20);

    (void)disk_ioctl(0, CTRL_SYNC, 0);   // waits for not-busy in most ports
    HAL_Delay(2);
}


/* Force a clean re-init/mount even if we thought we were mounted */
int SDCard_ForceReinit(void)
{
    s_sd_mounted = 0;        /* make SDCard_Init() do real work */
    /* give the card clocks with CS high before re-entering SPI mode */
    SDCard_Quiesce();
    return SDCard_Init();
}

static void _sd_spi_set_prescaler(uint32_t presc)
{
    HAL_SPI_DeInit(&hspi3);
    hspi3.Init.BaudRatePrescaler = presc;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Initialize SD card and mount FatFs (idempotent).
 * @return 0 on success; nonzero on failure.
 */
DSTATUS SDCard_Init(void)
{
    if (s_sd_mounted) {
        return 0;  /* already mounted */
    }

    /* SD CS must idle HIGH before any clocks; let power settle */
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(150);

    /* Slow SPI for identification (your diskio.c may re-speed later) */
    _sd_spi_set_prescaler(SPI_BAUDRATEPRESCALER_256);

    FRESULT fr = FR_INT_ERR;

    for (int attempt = 0; attempt < 3; ++attempt) {
        /* disk_initialize(0) returns DSTATUS; RES_OK == 0 */
        DSTATUS ds = disk_initialize(0);
        if (ds == RES_OK) {
            /* Mount drive 0 at default path "" */
            fr = f_mount(&SDFatFS, "", 1);
            if (fr == FR_OK) {
                printf("SDCard_Init: card mounted OK\r\n");
                s_sd_mounted = 1;

                /* Safer transfer speed on dupont/breadboard wiring */
                _sd_spi_set_prescaler(SPI_BAUDRATEPRESCALER_16); /* ~4–6 MHz, adjust if stable */
                return 0;
            } else {
                printf("SDCard_Init: f_mount failed (%u)\r\n", (unsigned)fr);
            }
        } else {
            printf("SDCard_Init: disk_initialize failed (0x%02X)\r\n", (unsigned)ds);
        }

        HAL_Delay(150);  /* give card time and retry */
    }

    /* still not mounted */
    return 1;
}


/* Optional helper if you want callers to check */
int SDCard_IsMounted(void) { return s_sd_mounted; }


/**
 * @brief  Simple write/read test: creates "test.txt", writes a line,
 *         reads it back, prints the result.
 */
void SDCard_TestFileIO(void)
{
    FIL file;
    FRESULT fr;
    UINT bw, br;
    char buf[64];

    /* Make sure card is mounted */
    if (SDCard_Init() != RES_OK) {
        printf("SDCard_TestFileIO: init failed\r\n");
        return;
    }

    /* Write */
    fr = f_open(&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("SDCard_TestFileIO: f_open write error (%u)\r\n", fr);
        return;
    }
    const char *msg = "This is actual data haha beep boop 0x12 THROTTLE IS YAMMING at like 80 percent \r\n";
    fr = f_write(&file, msg, strlen(msg), &bw);
    f_close(&file);
    if (fr != FR_OK || bw != strlen(msg)) {
        printf("SDCard_TestFileIO: f_write error (%u), bw=%u\r\n", fr, bw);
        return;
    }
    printf("SDCard_TestFileIO: wrote %u bytes\r\n", bw);

    /* Read */
    fr = f_open(&file, "test.txt", FA_READ);
    if (fr != FR_OK) {
        printf("SDCard_TestFileIO: f_open read error (%u)\r\n", fr);
        return;
    }
    fr = f_read(&file, buf, sizeof(buf)-1, &br);
    f_close(&file);
    if (fr != FR_OK) {
        printf("SDCard_TestFileIO: f_read error (%u)\r\n", fr);
        return;
    }
    buf[br] = '\0';
    printf("SDCard_TestFileIO: read %u bytes: %s", br, buf);
}




/*-----------------------------------------------------------------------
 * Return a fixed timestamp:
 *   year = 2025, month = 8, day = 7, time = 00:00:00
 *-----------------------------------------------------------------------*/
DWORD get_fattime(void) {
    return  ((DWORD)(2025 - 1980) << 25)    /* Year = 2025 */
          | (8 << 21)                       /* Month = August */
          | (7 << 16)                       /* Day = 7 */
          | (0 << 11)                       /* Hour = 0 */
          | (0 << 5)                        /* Min = 0 */
          | (0 >> 1);                       /* Sec = 0 (sec/2) */
}
