#ifndef BOOTLOADER_UPDATE_H
#define BOOTLOADER_UPDATE_H

#include <m2m_image_3A0.inc>       /* image array & length                 */
#include <stdint.h>
#include "programmer.h"          /* erase / write / get_flash_size       */

int8_t bootloader_update_flash(void);

#endif
