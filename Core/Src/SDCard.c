#include "SDCard.h"
#include "main.h"    /* brings in hspi3, SDCARD_CS_Pin/Port, and retargeted printf */
#include <string.h>  /* for strlen */
#include <stdio.h>   /* for printf */
#include "ff.h"
#include "stm32h5xx_hal.h"

static FATFS SDFatFS;  /* File system object */
extern SPI_HandleTypeDef hspi3;

/**
 * @brief  Initialize SD card and mount filesystem.
 */
DSTATUS SDCard_Init(void)
{
    DSTATUS stat = disk_initialize(0);
    if (stat != RES_OK) {
        printf("SDCard_Init: disk_initialize failed (0x%02X)\r\n", stat);
        return stat;
    }

    FRESULT fr = f_mount(&SDFatFS, "", 1);
    if (fr != FR_OK) {
        printf("SDCard_Init: f_mount failed (%u)\r\n", fr);
        return STA_NOINIT;
    }

    printf("SDCard_Init: card mounted OK\r\n");

    /* ----- now speed up SPI for data transfers ----- */
    HAL_SPI_DeInit(&hspi3);
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;   // ≈16 MHz
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }

    return RES_OK;
}

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
