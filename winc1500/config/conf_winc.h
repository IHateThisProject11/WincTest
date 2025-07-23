/**
 *  Board-specific configuration – WINC1500 on STM32H503 (Nucleo-64)
 *  ────────────────────────────────────────────────────────────────
 *  Wiring assumed (from your screenshot):
 *      WINC1500  <-->  MCU
 *      ----------------------------------------
 *      SCK      →  PB3   (SPI1_SCK)
 *      MISO     ←  PA6   (SPI1_MISO)
 *      MOSI     →  PA7   (SPI1_MOSI)
 *      nCS      →  PC5   (GPIO out)
 *      nIRQ     ←  PC4   (GPIO in , EXTI4)
 *      RESET_N  →  PB0   (GPIO out)
 *      CHIP_EN  →  PB1   (GPIO out)
 *
 *  If you re-route any pin, just edit the #defines below.
 */
#ifndef CONF_WINC_H_
#define CONF_WINC_H_

#include "stm32h5xx_hal.h"      /* HAL headers first */
#include "main.h"               /* Cube-generated handles */

/* --------------------------------------------------------------------------
 * SPI interface (SPI1 on APB2 @ 80 MHz) --> ~10 MHz SPI clock
 * -------------------------------------------------------------------------- */
#define CONF_WINC_SPI_HANDLE        hspi1
#define CONF_WINC_SPI               SPI1
#define CONF_WINC_SPI_BAUD_PRESCAL  SPI_BAUDRATEPRESCALER_8   /* 80/8 = 10 MHz */

/* Chip-select -------------------------------------------------------------- */
#define CONF_WINC_SPI_CS_PORT       GPIOC
#define CONF_WINC_SPI_CS_PIN        GPIO_PIN_5
#define CONF_WINC_CS_ASSERT()   HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                  CONF_WINC_SPI_CS_PIN, GPIO_PIN_RESET)
#define CONF_WINC_CS_DEASSERT() HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                  CONF_WINC_SPI_CS_PIN, GPIO_PIN_SET)

/* SCK / MISO / MOSI --------------------------------------------------------*/
#define CONF_WINC_SPI_SCK_PORT   GPIOA      /* PA5 now */
#define CONF_WINC_SPI_SCK_PIN    GPIO_PIN_5

#define CONF_WINC_SPI_MISO_PORT  GPIOA      /* PA6 */
#define CONF_WINC_SPI_MISO_PIN   GPIO_PIN_6
#define CONF_WINC_SPI_MOSI_PORT  GPIOA      /* PA7 */
#define CONF_WINC_SPI_MOSI_PIN   GPIO_PIN_7

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
 *  Compatibility aliases – satisfy hard-coded names inside nm_bsp_stm32h5.c
 * ----------------------------------------------------------------------------
 *  If your board really has 3 V3 / level-shifter control, map them here.
 *  Otherwise leave them on unused pins so the code compiles harmlessly.
 */
#define CONF_WINC_PIN_POWER_ENABLE          GPIO_PIN_8         /* PA8 is free */
#define CONF_WINC_PORT_POWER_ENABLE         GPIOA

#define CONF_WINC_PIN_LEVEL_SHIFTER_ENABLE  GPIO_PIN_9         /* PC9 free  */
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
#define NM_DEBUG    1
#if NM_DEBUG
  #define CONF_WINC_PRINTF   printf
#else
  #define CONF_WINC_PRINTF(...)
#endif

#endif /* CONF_WINC_H_ */
