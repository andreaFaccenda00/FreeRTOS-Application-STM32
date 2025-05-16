#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Log an event with timestamp, phase, vehicle counts and info.
 * @param  phase  String describing the current phase (e.g. "NS").
 * @param  info   Description of the event (e.g. "VERDE(PRIO)").
 * @param  S      Number of vehicles south (or generic count1).
 * @param  N      Number of vehicles north (or generic count2).
 * @param  E      Number of vehicles east (or generic count2).
 */
void LogEvent(const char* phase, const char* info, uint32_t S, uint32_t N, uint32_t E );

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
