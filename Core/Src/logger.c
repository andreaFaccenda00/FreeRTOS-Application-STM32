#include "logger.h"
#include "stm32g4xx.h"

void EnableDWT(void) {
    CoreDebug->DEMCR   |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT         = 0;
    DWT->CTRL         |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t GetTimestampUs(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void LogEventTS(const char* tag) {
    uint32_t us = GetTimestampUs();
    uint32_t ms = osKernelGetTickCount();
    printf("%7lu us | %5lu ms | %s\r\n",
           (unsigned long)us,
           (unsigned long)ms,
           tag);
}

void LogEventValueTS(const char* tag, uint32_t value) {
    uint32_t us = GetTimestampUs();
    uint32_t ms = osKernelGetTickCount();
    printf("%7lu us | %5lu ms | %s: %lu\r\n",
           (unsigned long)us,
           (unsigned long)ms,
           tag,
           (unsigned long)value);
}
