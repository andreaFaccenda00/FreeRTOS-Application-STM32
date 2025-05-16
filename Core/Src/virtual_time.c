#include "virtual_time.h"
#include "cmsis_os2.h"
#include <stddef.h>


static uint8_t virtualHour = 0;
static osTimerId_t virtualTimer = NULL;

static void VirtualTimeCallback(void *argument)
{
    (void)argument;
    virtualHour = (virtualHour + 1) % 24;
}

void InitVirtualTime(uint32_t tickIntervalMs)
{
    const osTimerAttr_t timerAttr = { .name = "VirtClock" };
    virtualTimer = osTimerNew(VirtualTimeCallback, osTimerPeriodic, NULL, &timerAttr);
    if (virtualTimer != NULL) {
        osTimerStart(virtualTimer, tickIntervalMs);
    }
}

uint8_t GetVirtualHour(void)
{
    return virtualHour;
}

bool IsNightMode(void)
{
    return (virtualHour >= 23U || virtualHour < 6U);
}
