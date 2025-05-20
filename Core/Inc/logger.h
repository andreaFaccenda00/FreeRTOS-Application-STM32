// logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include "cmsis_os2.h"
#include <stdio.h>

void EnableDWT(void);
uint32_t GetTimestampUs(void);
void LogEventTS(const char* tag);
void LogEventValueTS(const char* tag, uint32_t value);

#endif // LOGGER_H
