/**
  ******************************************************************************
  * @file    bootloader_update.c
  * @brief   WINC1500 SPI ROM-bootloader flash update routine
  ******************************************************************************
  */

#include "bootloader_update.h"
#include "conf_winc.h"           /* CHIP_EN / RESET pin defs */
#include "stm32h5xx_hal.h"
#include "programmer.h"          /* programmer_erase(), programmer_write() */
#include "m2m_image_3A0.h"       /* extern const uint8_t m2m_image_3A0[], m2m_image_3A0_len */

/**
 * @brief  Toggle CHIP_EN/RESET to enter the WINC1500’s ROM bootloader over SPI.
 */
static void winc_enter_rom_loader(void)
{
    /* De-assert CHIP_EN, then RESET */
    HAL_GPIO_WritePin(CONF_WINC_CHIP_EN_PORT, CONF_WINC_CHIP_EN_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(CONF_WINC_RESET_PORT,    CONF_WINC_RESET_PIN,   GPIO_PIN_RESET);
    HAL_Delay(5);

    /* Re-assert CHIP_EN, then release RESET */
    HAL_GPIO_WritePin(CONF_WINC_CHIP_EN_PORT, CONF_WINC_CHIP_EN_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(CONF_WINC_RESET_PORT,    CONF_WINC_RESET_PIN,   GPIO_PIN_SET);
    HAL_Delay(50);  /* Boot-ROM is now listening on SPI */
}

/**
 * @brief  One-time SPI flash update: erase, program, (optional verify).
 * @retval M2M_SUCCESS on success, M2M_ERR_FAIL on any failure.
 */
int8_t bootloader_update_flash(void)
{
    /* Put WINC1500 into SPI ROM-loader mode */
    winc_enter_rom_loader();

    /* Erase entire flash */
    if (programmer_erase(0, programmer_get_flash_size(), NULL) != M2M_SUCCESS)
    {
        return M2M_ERR_FAIL;
    }

    /* Program the new firmware image into flash */
    if (programmer_write((uint8_t *)m2m_image_3A0, 0, m2m_image_3A0_len, NULL) != M2M_SUCCESS)
    {
        return M2M_ERR_FAIL;
    }

    /* (Optional) Read-back & verify here */

    return M2M_SUCCESS;
}
