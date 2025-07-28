#include "bootloader_update.h"
#include "conf_winc.h"           /* CHIP_EN / RESET pin defs */
#include "stm32h5xx_hal.h"

/* Reset sequence that leaves the WINC1500 in ROM-bootloader mode (SPI) */
static void winc_enter_rom_loader(void)
{
    HAL_GPIO_WritePin(CONF_WINC_CHIP_EN_PORT, CONF_WINC_CHIP_EN_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(CONF_WINC_RESET_PORT,    CONF_WINC_RESET_PIN,   GPIO_PIN_RESET);
    HAL_Delay(5);

    /* Enable, then release RESET */
    HAL_GPIO_WritePin(CONF_WINC_CHIP_EN_PORT, CONF_WINC_CHIP_EN_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(CONF_WINC_RESET_PORT,    CONF_WINC_RESET_PIN,   GPIO_PIN_SET);
    HAL_Delay(50);        /* boot-ROM is now listening on SPI */
}

int8_t bootloader_update_flash(void)
{
    winc_enter_rom_loader();

    /* Erase full flash, then program new image */
    if (programmer_erase(0, programmer_get_flash_size(), NULL) != M2M_SUCCESS)   /* macros → spi_flash_* */
:contentReference[oaicite:5]{index=5}
        return M2M_ERR_FAIL;

    if (programmer_write((uint8_t *)m2m_image_3A0, 0, m2m_image_3A0_len, NULL) != M2M_SUCCESS)
        return M2M_ERR_FAIL;

    /* optional: programmer_read / verify … */

    return M2M_SUCCESS;
}
