#include "main.h"      // for IWDG register definitions via stm32g4xx_hal_conf.h
#include "iwdg.h"
#include "FreeRTOS.h"  // for configUSE_IDLE_HOOK

#define IWDG_KR_KEY_ENABLE_ACCESS   0x5555U
#define IWDG_KR_KEY_START           0xCCCCU
#define IWDG_KR_KEY_RELOAD          0xAAAAU

void MX_IWDG_Init(void)
{
    // 1) Enable write access to PR and RLR
    IWDG->KR = IWDG_KR_KEY_ENABLE_ACCESS;
    // 2) Set prescaler to /256 (PR = 6)
    IWDG->PR = 6U;
    // 3) Set reload counter to maximum (12-bit → 0xFFF)
    IWDG->RLR = 0x0FFFU;
    // 4) Reload immediately
    IWDG->KR = IWDG_KR_KEY_RELOAD;
    // 5) Start the watchdog
    IWDG->KR = IWDG_KR_KEY_START;
}

void IWDG_Refresh(void)
{
    // Reload the counter to prevent system reset
    IWDG->KR = IWDG_KR_KEY_RELOAD;
}

/**
 * @brief  FreeRTOS Idle Hook: invoked when no other task is ready.
 *         Automatically refreshes the IWDG.
 */
void vApplicationIdleHook(void)
{
    IWDG_Refresh();
}
