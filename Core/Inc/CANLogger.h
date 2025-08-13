#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Returns 0 on success; negative on error.
 * Opens/creates 0:/can_log.csv (append), writes header if file was empty,
 * configures FDCAN1 filter + notifications, and starts FDCAN1.
 */
int CANLogger_Init(void);

/* Call periodically (e.g., every 10–100 ms) to flush buffered lines. */
void CANLogger_Tick(void);

/* Optional: enable/disable internal loopback for bench testing (call before Init). */
void CANLogger_SetLoopback(bool enable);

/* True after Init succeeded and file is open. */
bool CANLogger_Ready(void);

int  CANLogger_Suspend(void);

int CANLogger_Resume(void);
