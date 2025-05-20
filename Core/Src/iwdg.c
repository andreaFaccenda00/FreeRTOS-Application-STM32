#include <stdio.h>
#include "iwdg.h"

void MX_IWDG_Init(void)
{
    IWDG->KR = IWDG_KR_KEY_ENABLE_ACCESS;
    IWDG->PR = 6U;
    IWDG->RLR = 0x0FFFU;
    IWDG->KR = IWDG_KR_KEY_RELOAD;
    IWDG->KR = IWDG_KR_KEY_START;
}

void IWDG_Refresh(void)
{
    IWDG->KR = IWDG_KR_KEY_RELOAD;
}

void vApplicationIdleHook(void)
{

	static uint32_t last_cnt;
	uint32_t now = DWT->CYCCNT;
	uint32_t idle_cycles = now - last_cnt;
	last_cnt = now;
	LogEventValueTS("Idle_cycles", idle_cycles);
	IWDG_Refresh();
}



