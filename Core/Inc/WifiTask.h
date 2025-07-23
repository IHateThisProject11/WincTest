// WifiTask.h

#ifndef WIFITASK_H
#define WIFITASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  One-time initialization of the WINC1500 driver and AP connection.
 *         Must be called once before entering the main loop.
 */
void WifiTask_Init(void);

/**
 * @brief  Call as often as possible from the super-loop to service the
 *         WINC1500 driver state machine.
 */
void WifiTask_Tick(void);

#ifdef __cplusplus
}
#endif

#endif // WIFITASK_H
