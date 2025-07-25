/**
 *  Board-specific configuration – WINC1500 on STM32H503 (Nucleo-64)
 *  ────────────────────────────────────────────────────────────────
 *  Wiring assumed:
 *      WINC1500  <-->  MCU
 *      SCK      →  PB3   (SPI1_SCK)
 *      MISO     ←  PA6   (SPI1_MISO)
 *      MOSI     →  PA7   (SPI1_MOSI)
 *      nCS      →  PC5   (GPIO out)
 *      nIRQ     ←  PC4   (GPIO in , EXTI4)
 *      RESET_N  →  PB0   (GPIO out)
 *      CHIP_EN  →  PB1   (GPIO out)
 */

#ifndef CONF_WINC_H_
#define CONF_WINC_H_

#include "stm32h5xx_hal.h"      /* HAL headers first */
#include "main.h"               /* Cube-generated handles */

/* --------------------------------------------------------------------------
 * SPI interface (SPI1 on APB2 @80 MHz → 10 MHz SPI clock)
 * -------------------------------------------------------------------------- */
#define CONF_WINC_SPI_HANDLE        hspi1
#define CONF_WINC_SPI               SPI1
#define SPI_WIFI_HANDLE             CONF_WINC_SPI_HANDLE
#define CONF_WINC_SPI_BAUD_PRESCALER  SPI_BAUDRATEPRESCALER_2   /* <- 1.7.7 expects this name */
#define CONF_WINC_SPI_BAUD_PRESCAL   CONF_WINC_SPI_BAUD_PRESCALER /* keep old alias, harmless */

#define CONF_WINC_SPI_LOW_BAUD_PRESCALER  SPI_BAUDRATEPRESCALER_8

/* Chip-select -------------------------------------------------------------- */
#define CONF_WINC_SPI_CS_PORT       GPIOC
#define CONF_WINC_SPI_CS_PIN        GPIO_PIN_5
#define CONF_WINC_CS_ASSERT()       HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                   CONF_WINC_SPI_CS_PIN, GPIO_PIN_RESET)
#define CONF_WINC_CS_DEASSERT()     HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                   CONF_WINC_SPI_CS_PIN, GPIO_PIN_SET)

/* convenience used by the bus-wrapper ----------------------------------- */
#define SPI_WIFI_HANDLE                CONF_WINC_SPI_HANDLE
#define SPI_WIFI_CS_GPIO_PORT          CONF_WINC_SPI_CS_PORT
#define SPI_WIFI_CS_PIN                CONF_WINC_SPI_CS_PIN
/* SCK / MISO / MOSI -------------------------------------------------------- */
#define CONF_WINC_SPI_SCK_PORT      GPIOA
#define CONF_WINC_SPI_SCK_PIN       GPIO_PIN_5

#define CONF_WINC_SPI_MISO_PORT     GPIOA
#define CONF_WINC_SPI_MISO_PIN      GPIO_PIN_6

#define CONF_WINC_SPI_MOSI_PORT     GPIOA
#define CONF_WINC_SPI_MOSI_PIN      GPIO_PIN_7

/* RESET_N & CHIP_EN -------------------------------------------------------- */
#define CONF_WINC_RESET_PORT        GPIOB
#define CONF_WINC_RESET_PIN         GPIO_PIN_0

#define CONF_WINC_CHIP_EN_PORT      GPIOB
#define CONF_WINC_CHIP_EN_PIN       GPIO_PIN_1

/* nIRQ --------------------------------------------------------------------- */
#define CONF_WINC_IRQ_PORT          GPIOC
#define CONF_WINC_IRQ_PIN           GPIO_PIN_4
#define CONF_WINC_IRQ_EXTI_IRQn     EXTI4_IRQn                 /* PC4 ↔ EXTI4 */

/* ----------------------------------------------------------------------------
 *  Compatibility aliases – satisfy hard-coded names in nm_bsp_stm32h5.c & bus_wrapper
 * ----------------------------------------------------------------------------
 */
/* core SPI bus */
#define SPI_WIFI                  CONF_WINC_SPI
#define SPI_WIFI_CLK_ENABLE()     __HAL_RCC_SPI1_CLK_ENABLE()

/* chip-select */
#define SPI_WIFI_CS_GPIO_PORT     CONF_WINC_SPI_CS_PORT
#define SPI_WIFI_CS_PIN           CONF_WINC_SPI_CS_PIN

/* data pins */
#define SPI_WIFI_SCK_GPIO_PORT    CONF_WINC_SPI_SCK_PORT
#define SPI_WIFI_SCK_PIN          CONF_WINC_SPI_SCK_PIN

#define SPI_WIFI_MISO_GPIO_PORT   CONF_WINC_SPI_MISO_PORT
#define SPI_WIFI_MISO_PIN         CONF_WINC_SPI_MISO_PIN

#define SPI_WIFI_MOSI_GPIO_PORT   CONF_WINC_SPI_MOSI_PORT
#define SPI_WIFI_MOSI_PIN         CONF_WINC_SPI_MOSI_PIN

/* AF mapping for PB3/PB4/PB5 → SPI1 */
#define SPI3_WIFI_AF              GPIO_AF5_SPI1

/* GPIO speed enum alias */
#define GPIO_SPEED_HIGH           GPIO_SPEED_FREQ_HIGH
#define GPIO_SPEED_MEDIUM         GPIO_SPEED_FREQ_MEDIUM
#define GPIO_SPEED_LOW            GPIO_SPEED_FREQ_LOW
#define GPIO_SPEED_VERY_HIGH      GPIO_SPEED_FREQ_VERY_HIGH

/* power & level-shifter (unused on this board but needed by BSP) */
#define CONF_WINC_PIN_POWER_ENABLE          GPIO_PIN_8   /* PA8 */
#define CONF_WINC_PORT_POWER_ENABLE         GPIOA

#define CONF_WINC_PIN_LEVEL_SHIFTER_ENABLE  GPIO_PIN_9   /* PC9 */
#define CONF_WINC_PORT_LEVEL_SHIFTER_ENABLE GPIOC

#define CONF_WINC_PIN_CHIP_ENABLE           CONF_WINC_CHIP_EN_PIN
#define CONF_WINC_PORT_CHIP_ENABLE          CONF_WINC_CHIP_EN_PORT

#define CONF_WINC_PIN_RESET                 CONF_WINC_RESET_PIN
#define CONF_WINC_PORT_RESET                CONF_WINC_RESET_PORT

#define CONF_WINC_SPI_INT_PIN               CONF_WINC_IRQ_PIN
#define CONF_WINC_SPI_INT_PORT              CONF_WINC_IRQ_PORT
#define CONF_WINC_EXTI_IRQN                 CONF_WINC_IRQ_EXTI_IRQn

/* --------------------------------------------------------------------------
 * Debug printing (1 = printf traces)
 * -------------------------------------------------------------------------- */
#define CONF_WINC_PRINTF   printf
#define CONF_WINC_DEBUG    1

#define NM_DEBUG    1
#if NM_DEBUG
  #define CONF_WINC_PRINTF   printf
#else
  #define CONF_WINC_PRINTF(...)
#endif



/* also map chip-select lines if needed by the wrapper */
#define SPI_WIFI_CS_GPIO_PORT   CONF_WINC_SPI_CS_PORT
#define SPI_WIFI_CS_PIN         CONF_WINC_SPI_CS_PIN
/* ─────────────────────────────────────────────────────────────────────────── */

#endif /* CONF_WINC_H_ */
