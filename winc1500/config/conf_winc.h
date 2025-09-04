/**
 *  Board-specific configuration – WINC1500 on STM32H503 (Nucleo-64)
 *  ────────────────────────────────────────────────────────────────
 *  Wiring assumed:
 *      WINC1500  <-->  MCU
 *      SCK      →  PA5   (SPI1_SCK)
 *      MISO     ←  PA6   (SPI1_MISO)
 *      MOSI     →  PA7   (SPI1_MOSI)
 *      nCS      →  PC5   (GPIO out)
 *      nIRQ     ←  PC4   (GPIO in , EXTI4)
 *      RESET_N  →  PB0   (GPIO out)
 *      CHIP_EN  →  PB1   (GPIO out)
 */

#ifndef CONF_WINC_H_
#define CONF_WINC_H_

#include "stm32h5xx_hal.h"
#include "main.h"           /* for extern hspi1 */

extern SPI_HandleTypeDef hspi1;

/* --------------------------------------------------------------------------
 * SPI interface (SPI1 on APB2 @80 MHz → 10 MHz SPI clock)
 * -------------------------------------------------------------------------- */
#define CONF_WINC_USE_SPI               (1)
#define CONF_WINC_SPI_HANDLE            hspi1
#define CONF_WINC_SPI                   SPI1
#define CONF_WINC_SPI_BAUD_PRESCALER    SPI_BAUDRATEPRESCALER_2
#define CONF_WINC_SPI_LOW_BAUD_PRESCALER SPI_BAUDRATEPRESCALER_8

/* Chip-select -------------------------------------------------------------- */
#define CONF_WINC_SPI_CS_PORT           GPIOC
#define CONF_WINC_SPI_CS_PIN            GPIO_PIN_5
#define CONF_WINC_CS_ASSERT()           HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                        CONF_WINC_SPI_CS_PIN, GPIO_PIN_RESET)
#define CONF_WINC_CS_DEASSERT()         HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                        CONF_WINC_SPI_CS_PIN, GPIO_PIN_SET)

/* Data pins --------------------------------------------------------------- */
#define CONF_WINC_SPI_SCK_PORT          GPIOA
#define CONF_WINC_SPI_SCK_PIN           GPIO_PIN_5
#define CONF_WINC_SPI_MISO_PORT         GPIOA
#define CONF_WINC_SPI_MISO_PIN          GPIO_PIN_6
#define CONF_WINC_SPI_MOSI_PORT         GPIOA
#define CONF_WINC_SPI_MOSI_PIN          GPIO_PIN_7

/* RESET_N & CHIP_EN ------------------------------------------------------- */
#define CONF_WINC_RESET_PORT            GPIOB
#define CONF_WINC_RESET_PIN             GPIO_PIN_0
#define CONF_WINC_CHIP_EN_PORT          GPIOB
#define CONF_WINC_CHIP_EN_PIN           GPIO_PIN_1

/* nIRQ (EXTI) ------------------------------------------------------------- */
#define CONF_WINC_IRQ_PORT              GPIOC
#define CONF_WINC_IRQ_PIN               GPIO_PIN_4
#define CONF_WINC_EXTI_IRQN             EXTI4_IRQn

/* ----------------------------------------------------------------------------
 *  Compatibility aliases for nm_bsp_stm32h5.c & bus_wrapper
 * ----------------------------------------------------------------------------
 */
/* core SPI bus */
#define SPI_WIFI_HANDLE                 CONF_WINC_SPI_HANDLE
#define SPI_WIFI                        CONF_WINC_SPI
#define SPI_WIFI_CLK_ENABLE()           __HAL_RCC_SPI1_CLK_ENABLE()

/* chip-select */
#define SPI_WIFI_CS_GPIO_PORT           CONF_WINC_SPI_CS_PORT
#define SPI_WIFI_CS_PIN                 CONF_WINC_SPI_CS_PIN

/* data pins */
#define SPI_WIFI_SCK_GPIO_PORT          CONF_WINC_SPI_SCK_PORT
#define SPI_WIFI_SCK_PIN                CONF_WINC_SPI_SCK_PIN
#define SPI_WIFI_MISO_GPIO_PORT         CONF_WINC_SPI_MISO_PORT
#define SPI_WIFI_MISO_PIN               CONF_WINC_SPI_MISO_PIN
#define SPI_WIFI_MOSI_GPIO_PORT         CONF_WINC_SPI_MOSI_PORT
#define SPI_WIFI_MOSI_PIN               CONF_WINC_SPI_MOSI_PIN

/* AF mapping for all SPI1 lines */
#define SPI_WIFI_AF                     GPIO_AF5_SPI1

/* optional power & level-shifter (unused but declared) */
#define CONF_WINC_PORT_POWER_ENABLE     GPIOA
#define CONF_WINC_PIN_POWER_ENABLE      GPIO_PIN_8
#define CONF_WINC_PORT_LVL_SHIFT        GPIOC
#define CONF_WINC_PIN_LVL_SHIFT         GPIO_PIN_9

/* ----------------------------------------------------------------------------
 *  The missing defines your BSP code expects:
 * ----------------------------------------------------------------------------
 */
#define CONF_WINC_PIN_CHIP_ENABLE           CONF_WINC_CHIP_EN_PIN
#define CONF_WINC_PORT_CHIP_ENABLE          CONF_WINC_CHIP_EN_PORT

#define CONF_WINC_PIN_RESET                 CONF_WINC_RESET_PIN
#define CONF_WINC_PORT_RESET                CONF_WINC_RESET_PORT

#define CONF_WINC_PIN_LEVEL_SHIFTER_ENABLE  CONF_WINC_PIN_LVL_SHIFT
#define CONF_WINC_PORT_LEVEL_SHIFTER_ENABLE CONF_WINC_PORT_LVL_SHIFT

#define CONF_WINC_SPI_INT_PIN               CONF_WINC_IRQ_PIN
#define CONF_WINC_SPI_INT_PORT              CONF_WINC_IRQ_PORT

/* nm_bus_wrapper wants this legacy name */
#define SPI3_WIFI_AF                         SPI_WIFI_AF

/* --------------------------------------------------------------------------
 * Debug printing (1 = enabled)
 * -------------------------------------------------------------------------- */
#define CONF_WINC_DEBUG                 (1)
#if CONF_WINC_DEBUG
  #define CONF_WINC_PRINTF(...)         printf(__VA_ARGS__)
#else
  #define CONF_WINC_PRINTF(...)
#endif

#endif /* CONF_WINC_H_ */
