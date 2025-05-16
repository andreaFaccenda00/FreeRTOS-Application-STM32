#include "logger.h"
#include <stdio.h>
#include "cmsis_os2.h"

void LogEvent(const char* phase, const char* info, uint32_t S, uint32_t N, uint32_t E)
{
    uint32_t seconds = osKernelGetTickCount() / 1000U;
    // Format: " 123s | PHASE | S=xx N=yy | INFO"
    printf("%4lus | %s | S=%2lu N=%2lu | %s\r\n",
           (unsigned long)seconds,
           phase,
           (unsigned long)S,
           (unsigned long)N,
           info);
}
