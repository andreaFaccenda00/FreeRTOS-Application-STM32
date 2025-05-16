#ifndef VIRTUAL_TIME_H
#define VIRTUAL_TIME_H

#include <stdint.h>
#include <stdbool.h>

void InitVirtualTime(uint32_t tickIntervalMs);
uint8_t GetVirtualHour(void);
bool IsNightMode(void);

#endif // VIRTUAL_TIME_H
