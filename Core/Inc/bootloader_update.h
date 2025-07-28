#ifndef BOOTLOADER_UPDATE_H
#define BOOTLOADER_UPDATE_H

#include <stdint.h>
#include "programmer.h"          /* erase / write / get_flash_size       */
#include "m2m_image_3A0.h"       /* image array & length                 */

int8_t bootloader_update_flash(void);

#endif
