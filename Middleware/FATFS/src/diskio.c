#include "ff.h"        /* FatFs API (f_mount, etc.) */
#include "diskio.h"    /* lower‐layer APIs for Chan’s FAT driver */
#include "main.h"      /* brings in HAL + your hspi3 & pin defines */


extern SPI_HandleTypeDef hspi3; /* your SPI3 handle */

/* Block (sector) vs byte addressing flag: 1 = block addressing (SDHC/SDXC) */
static uint8_t CardType = 0;


#define SDCARD_CS_LOW()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SDCARD_CS_HIGH() HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

static DSTATUS Stat = STA_NOINIT;

/* Send and receive a single byte over SPI */
static uint8_t spi_xfer(uint8_t data) {
    uint8_t resp;
    HAL_SPI_TransmitReceive(&hspi3, &data, &resp, 1, HAL_MAX_DELAY);
    return resp;
}

/* Send N bytes of dummy 0xFF clocks */
static void send_initial_clock_train(void) {
    uint8_t i;
    SDCARD_CS_HIGH();
    for (i = 0; i < 10; i++) {
        spi_xfer(0xFF);
    }
}

/* Wait for card to respond (R1 format), timeout ~500µs */
static uint8_t wait_r1(void) {
    uint8_t r;
    uint32_t timeout = HAL_GetTick() + 1;
    do {
        r = spi_xfer(0xFF);
    } while ((r & 0x80) && (HAL_GetTick() < timeout));
    return r;
}

/* Send a command packet (CMDn) and return R1 */
static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t buf[6], crc;

    /* ACMD<n> is CMD55 + CMD<n> */
    if (cmd & 0x80) {
        cmd &= 0x7F;
        if (send_cmd(55, 0) > 1) return 0xFF;
    }

    /* select & give a dummy before */
    SDCARD_CS_LOW();
    spi_xfer(0xFF);

    buf[0] = 0x40 | cmd;
    buf[1] = arg >> 24;
    buf[2] = arg >> 16;
    buf[3] = arg >>  8;
    buf[4] = arg;
    /* CRC: only valid for CMD0 and CMD8 */
    crc = (cmd == 0) ? 0x95 : (cmd == 8 ? 0x87 : 0x01);
    buf[5] = crc;

    HAL_SPI_Transmit(&hspi3, buf, 6, HAL_MAX_DELAY);
    return wait_r1();
}

/* Receive a data block into buff, length=512, return 1 on OK */
static uint8_t rcvr_datablock(uint8_t *buff) {
    uint8_t token;
    uint32_t timeout = HAL_GetTick() + 100;

    /* wait for data token 0xFE */
    do {
        token = spi_xfer(0xFF);
    } while ((token == 0xFF) && (HAL_GetTick() < timeout));
    if (token != 0xFE) return 0;

    /* read 512 bytes */
    for (uint16_t i = 0; i < 512; i++) {
        buff[i] = spi_xfer(0xFF);
    }
    /* discard CRC */
    spi_xfer(0xFF);
    spi_xfer(0xFF);
    return 1;
}

/*-----------------------------------------------------------------------*/
/* Initialize Drive                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv) {
    uint8_t r1;
    uint16_t retry = 0xFFF;

    if (pdrv != 0) return STA_NOINIT;

    send_initial_clock_train();

    /* CMD0: go idle */
    do {
        r1 = send_cmd(0, 0);
    } while ((r1 != 0x01) && --retry);
    if (r1 != 0x01) goto init_fail;

    /* CMD8: voltage check */
    r1 = send_cmd(8, 0x1AA);
    if (r1 & 0x04)   /* illegal cmd = SDSC v1 */
        retry = 0xFFF;
    else {
        /* read rest of R7 */
        spi_xfer(0xFF); spi_xfer(0xFF);
        spi_xfer(0xFF); spi_xfer(0xFF);
    }

    /* ACMD41: init, HCS bit */
    do {
        r1 = send_cmd(0x80|41, 0x40000000);
    } while ((r1 != 0x00) && --retry);
    if (r1 != 0x00) goto init_fail;

    /* CMD58: read OCR */
    if (send_cmd(58, 0) != 0x00) goto init_fail;

    /* Read OCR bytes into an array */
    uint8_t ocr[4];
    for (int i = 0; i < 4; i++) {
        ocr[i] = spi_xfer(0xFF);
    }

    /* CCS bit (bit6 of OCR[0]) = 1 means SDHC/SDXC (block addressing) */
    if (ocr[0] & 0x40) {
        CardType = 1;
    } else {
        CardType = 0;
    }


    Stat &= ~STA_NOINIT;
    SDCARD_CS_HIGH();
    spi_xfer(0xFF);
    return Stat;

init_fail:
    SDCARD_CS_HIGH();
    spi_xfer(0xFF);
    return STA_NOINIT;
}


/**
 *  Return the argument to send_cmd():
 *   - Block address if SDHC/SDXC
 *   - Byte address if legacy SDSC
 */
static inline uint32_t sd_addr(LBA_t sector)
{
    return CardType ? sector        /* SDHC/SDXC: block address */
                    : sector * 512;  /* SDSC:    byte  address */
}

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv) {
    return (pdrv == 0 && !(Stat & STA_NOINIT)) ? 0 : STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Read Sector(s) – now supports count > 1 by looping CMD17              */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv || !count)      return RES_PARERR;
    if (Stat & STA_NOINIT)   return RES_NOTRDY;

    for (UINT i = 0; i < count; i++) {
        /* Send CMD17 for each sector */
        if (send_cmd(17, sd_addr(sector + i)) != 0) {
            /* De-select and fail */
            SDCARD_CS_HIGH();
            spi_xfer(0xFF);
            return RES_ERROR;
        }
        /* Read one 512-byte block */
        if (!rcvr_datablock(buff + (i * 512))) {
            SDCARD_CS_HIGH();
            spi_xfer(0xFF);
            return RES_ERROR;
        }

        /* Finish this transaction cleanly before next sector */
        SDCARD_CS_HIGH();
        spi_xfer(0xFF);
    }

    return RES_OK;
}

#if FF_FS_READONLY == 0
/*-----------------------------------------------------------------------*/
/* Write Sector(s) – now supports count > 1 by looping CMD24             */
/*-----------------------------------------------------------------------*/
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv || !count)      return RES_PARERR;
    if (Stat & STA_NOINIT)   return RES_NOTRDY;

    for (UINT i = 0; i < count; i++) {
        /* CMD24: write single block */
        if (send_cmd(24, sd_addr(sector + i)) != 0) {
            SDCARD_CS_HIGH();
            spi_xfer(0xFF);
            return RES_ERROR;
        }

        /* One byte gap before token per many app notes */
        spi_xfer(0xFF);

        /* Data token */
        spi_xfer(0xFE);

        /* Push 512 bytes */
        HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi3,
                                                (uint8_t const*)(buff + (i * 512)),
                                                512, HAL_MAX_DELAY);
        if (st != HAL_OK) {
            SDCARD_CS_HIGH();
            spi_xfer(0xFF);
            return RES_ERROR;
        }

        /* Dummy CRC (not checked in SPI mode) */
        spi_xfer(0xFF);
        spi_xfer(0xFF);

        /* Data response: 0bXXX0_0101 = accepted */
        uint8_t resp = spi_xfer(0xFF);
        if ((resp & 0x1F) != 0x05) {
            SDCARD_CS_HIGH();
            spi_xfer(0xFF);
            return RES_ERROR;
        }

        /* Busy wait until card releases (returns 0xFF) */
        uint32_t tmo = HAL_GetTick() + 500;
        while (spi_xfer(0xFF) == 0x00) {
            if (HAL_GetTick() > tmo) {
                SDCARD_CS_HIGH();
                spi_xfer(0xFF);
                return RES_ERROR;
            }
        }

        /* Finish this transaction before next sector */
        SDCARD_CS_HIGH();
        spi_xfer(0xFF);
    }

    return RES_OK;
}
#endif /* FF_FS_READONLY == 0 */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv) return RES_PARERR;
    switch (cmd) {
        case CTRL_SYNC:
            /* nothing to do */
            return RES_OK;
        case GET_SECTOR_COUNT:
            /* TODO: implement if you read CSD */
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 8;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}
