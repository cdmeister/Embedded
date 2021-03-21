#ifndef DEGUG_PRINT
#define DEBUG_PRINT

#include "debug_helper.h"

uint32_t stm32_vsprintf(char * buffer, const char * fmt, va_list args);
void stm32_printf(const char * fmt, ...);

#endif /*DEBUG_PRINT*/
