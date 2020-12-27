#ifndef DEGUG_HELPER
#define DEBUG_HELPER

#include "stdint.h"
#include "stdarg.h"
#include "stddef.h"

char * stm32_strcpy(char * pSrc, char * pDest);
char * stm32_itoa(int num, char * str, uint32_t base);
char * stm32_convertToHex(uint32_t num, char * str, uint32_t alphaCase);
char * stm32_floatToString(float num, char * str);



#endif /*DEBUG_HELPER*/
