#include "logger.h"
#include <stdio.h>
#include "cmsis_os2.h"

void LogEvent(const char* phase,
              const char* info,
              uint32_t S,
              uint32_t N,
              uint32_t E)
{
    uint32_t ticks   = osKernelGetTickCount();    // in ms
    uint32_t seconds = ticks / 1000U;
    uint32_t ms      = ticks % 1000U;

    // Format: " 123.456s | PHASE | S=xx N=yy E=zz | INFO"
    printf("%4lu.%03lu s | %s | S=%2lu N=%2lu E=%2lu | %s\r\n",
           (unsigned long)seconds,
           (unsigned long)ms,
           phase,
           (unsigned long)S,
           (unsigned long)N,
           (unsigned long)E,
           info);
}
