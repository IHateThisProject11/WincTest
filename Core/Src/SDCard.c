#include "ff.h"            // for FATFS, FRESULT, f_mount()
#include "diskio.h"        // for disk_initialize()

static FATFS SDFatFS;       // File system object for the SD card

/**
 * @brief  Initialize SD card and mount filesystem.
 * @retval DSTATUS: RES_OK on success, or SD status error code.
 */
DSTATUS SDCard_Init(void)
{
    DSTATUS stat = disk_initialize(0);           // Chan’s API init :contentReference[oaicite:1]{index=1}
    if (stat != RES_OK) return stat;

    FRESULT fr = f_mount(&SDFatFS, "", 1);       // mount now :contentReference[oaicite:2]{index=2}
    if (fr != FR_OK) return STA_NOINIT;

    return RES_OK;
}
