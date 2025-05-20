#ifndef IWDG_H
#define IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* ⏱️ Independent Watchdog (IWDG) Interface                                   */
/*----------------------------------------------------------------------------*/
/*  Provides basic control over the STM32 Independent Watchdog in bare-metal  */
/*  and RTOS contexts. Ensures system recovery in case of hang or deadlock.   */
/*============================================================================*/

#include "stm32g4xx.h"   // 🧩 STM32 device-specific definitions for IWDG
#include "logger.h"      // 📝 For timestamped event logging (LogEventTS)

/*----------------------------------------------------------
 * 🔑 Watchdog Register Keys
 * Used to unlock, refresh, and start the IWDG via KR register
 *---------------------------------------------------------*/
#define IWDG_KR_KEY_ENABLE_ACCESS   0x5555U  // Unlock access to PR & RLR
#define IWDG_KR_KEY_RELOAD          0xAAAAU  // Reload the watchdog counter
#define IWDG_KR_KEY_START           0xCCCCU  // Start the watchdog

/*----------------------------------------------------------
 * 🛠️ Watchdog Initialization
 * Configures IWDG in bare-metal mode:
 *  • Prescaler: /256
 *  • Reload value: max (0xFFF)
 *  • Starts watchdog after setup
 *---------------------------------------------------------*/
void MX_IWDG_Init(void);

/*----------------------------------------------------------
 * 🔄 Watchdog Refresh
 * Reloads the IWDG counter to prevent system reset.
 * Should be called periodically during normal operation.
 *---------------------------------------------------------*/
void IWDG_Refresh(void);

/*----------------------------------------------------------
 * 💤 FreeRTOS Idle Hook
 * Automatically called when the RTOS is idle.
 * Refreshes the watchdog and logs idle time.
 *---------------------------------------------------------*/
void vApplicationIdleHook(void);

#ifdef __cplusplus
}
#endif

#endif // IWDG_H
