#ifndef SDCARD_H
#define SDCARD_H

#include "ff.h"       /* FatFs types (FRESULT, FIL, etc.) */
#include "diskio.h"   /* disk_initialize(), DSTATUS */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize SD card interface and mount filesystem.
 * @retval DSTATUS  RES_OK (0) on success, or an error code.
 */
DSTATUS SDCard_Init(void);

/**
 * @brief  Create “test.txt”, write a line, read it back, and print results.
 *         Relies on printf() being routed to your debug console.
 */
void SDCard_TestFileIO(void);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_H */
