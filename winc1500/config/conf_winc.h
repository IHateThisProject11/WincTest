/**
 * @file    conf_winc.h
 * @brief   Board-specific configuration for WINC1500 + STM32H503RB
 *
 * NOTE:  Adjust the GPIO pin/port definitions to match your wiring!
 *        The defaults assume:
 *          – WINC1500 SPI on SPI1 (PA5/6/7 + PA4 CS)
 *          – IRQ  : PB0  (EXTI0)
 *          – RESET: PC7
 *          – EN   : PC8  (a.k.a. CHIP_EN / CE)
 */

#ifndef CONF_WINC_H_
#define CONF_WINC_H_

#include "stm32h5xx_hal.h"   /* HAL headers first */
#include "main.h"            /* Cube-generated handles */

/* --------------------------------------------------------------------------
 * SPI interface selection
 * -------------------------------------------------------------------------- */
#define CONF_WINC_SPI_HANDLE        hspi1          /* extern SPI_HandleTypeDef */
#define CONF_WINC_SPI               SPI1
#define CONF_WINC_SPI_BAUD_PRESCAL  SPI_BAUDRATEPRESCALER_8  /* ~=10 MHz @ 80 MHz APB */

/* --------------------------------------------------------------------------
 * Chip-select (CS) line
 * -------------------------------------------------------------------------- */
#define CONF_WINC_SPI_CS_PORT       GPIOA
#define CONF_WINC_SPI_CS_PIN        GPIO_PIN_4
#define CONF_WINC_CS_ASSERT()       HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                      CONF_WINC_SPI_CS_PIN, GPIO_PIN_RESET)
#define CONF_WINC_CS_DEASSERT()     HAL_GPIO_WritePin(CONF_WINC_SPI_CS_PORT, \
                                                      CONF_WINC_SPI_CS_PIN, GPIO_PIN_SET)

/* --------------------------------------------------------------------------
 * RESET (NRST) and CHIP_ENABLE (CE) lines
 * -------------------------------------------------------------------------- */
#define CONF_WINC_RESET_PORT        GPIOC
#define CONF_WINC_RESET_PIN         GPIO_PIN_7

#define CONF_WINC_CHIP_EN_PORT      GPIOC
#define CONF_WINC_CHIP_EN_PIN       GPIO_PIN_8

/* --------------------------------------------------------------------------
 * Interrupt (IRQ) line
 * -------------------------------------------------------------------------- */
#define CONF_WINC_IRQ_PORT          GPIOB
#define CONF_WINC_IRQ_PIN           GPIO_PIN_0
#define CONF_WINC_IRQ_EXTI_IRQn     EXTI0_IRQn      /* matches PB0 */

/* ----------------------------------------------------------------------------
 * stubs & aliases for nm_bsp_stm32h5.c
 * ----------------------------------------------------------------------------
 */

/* power-enable (nm_bsp_deinit / init) */
#define CONF_WINC_PIN_POWER_ENABLE         GPIO_PIN_8   /* e.g. PA8 or whatever */
#define CONF_WINC_PORT_POWER_ENABLE        GPIOA        /* nm_bsp_stm32h5 hard-codes GPIOA here */

/* level-shifter enable (if your board has one) */
#define CONF_WINC_PIN_LEVEL_SHIFTER_ENABLE GPIO_PIN_9   /* e.g. PC9 */
#define CONF_WINC_PORT_LEVEL_SHIFTER_ENABLE GPIOC

/* chip-enable alias (nm_bsp wants these names) */
#define CONF_WINC_PIN_CHIP_ENABLE          CONF_WINC_CHIP_EN_PIN
#define CONF_WINC_PORT_CHIP_ENABLE         CONF_WINC_CHIP_EN_PORT

/* reset alias (nm_bsp may refer to these directly) */
#define CONF_WINC_PIN_RESET                CONF_WINC_RESET_PIN
#define CONF_WINC_PORT_RESET               CONF_WINC_RESET_PORT

/* interrupt alias (nm_bsp_register_isr / interrupt_ctrl) */
#define CONF_WINC_SPI_INT_PIN              CONF_WINC_IRQ_PIN
#define CONF_WINC_SPI_INT_PORT             CONF_WINC_IRQ_PORT
#define CONF_WINC_EXTI_IRQN                CONF_WINC_IRQ_EXTI_IRQn

/* --------------------------------------------------------------------------
 * Debug output
 * -------------------------------------------------------------------------- */
#define NM_DEBUG                    1               /* 0 = silent, 1 = printf */

#endif /* CONF_WINC_H_ */
