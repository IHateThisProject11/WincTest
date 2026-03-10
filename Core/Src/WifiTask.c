// WifiTask.c
#include "WifiTask.h"
#include "WifiApp.h"      // brings up nm_bsp + m2m_wifi_init
#include "m2m_wifi.h"
#include "stm32h5xx_hal.h"
#include "main.h"  // for IRQ_WINC_PIN_GPIO_Port / IRQ_WINC_PIN_Pin
extern volatile uint32_t g_irq_exti_fired, g_irq_bsp_isr, g_wifi_ticks;

void WifiTask_Init(void)
{
    // one-time init: GPIOs, SPI, WINC driver, connect to AP
    WifiApp_InitAP();
}

#include "main.h"  // for IRQ_WINC_PIN_GPIO_Port / IRQ_WINC_PIN_Pin
extern volatile uint32_t g_irq_exti_fired, g_irq_bsp_isr, g_wifi_ticks;

void WifiTask_Tick(void)
{
    g_wifi_ticks++;

    /* TEMPORARY failsafe: if nIRQ is asserted but EXTI didn’t wake us,
       call the BSP ISR from task context so events still flow. */
    uint8_t irq_level = HAL_GPIO_ReadPin(IRQ_WINC_PIN_GPIO_Port, IRQ_WINC_PIN_Pin);
    if (irq_level == GPIO_PIN_RESET) {
        printf("IRQ LOW! calling isr\r\n");
        nm_bsp_call_isr();
        g_irq_bsp_isr++;
    }

    m2m_wifi_handle_events(NULL);
}
