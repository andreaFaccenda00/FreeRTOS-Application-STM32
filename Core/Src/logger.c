#include "logger.h"
#include <stdio.h>
#include "stm32g4xx.h"
#include "system_stm32g4xx.h"
#include "cmsis_os2.h"

extern uint32_t SystemCoreClock;

static inline uint32_t GetTimestampUs(void)
{
    return (uint32_t)((uint64_t)DWT->CYCCNT * 1000000ULL / SystemCoreClock);
}

void LogEventTS(const char* tag)
{
    uint32_t us = GetTimestampUs();
    uint32_t ms = osKernelGetTickCount();

    printf("%7lu us | %5lu ms | %s\r\n",
           (unsigned long)us,
           (unsigned long)ms,
           tag);
}
