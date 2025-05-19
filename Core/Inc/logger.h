#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Logga un evento con timestamp preso da DWT->CYCCNT.
 * @param  tag    Tag string identifying the event (e.g. "ISR_PED", "NSTASK").
 */
void LogEventTS(const char* tag);

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
