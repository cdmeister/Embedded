#include "debug_print.h"

uint32_t stm32_vsprintf(char * buffer, const char * fmt, va_list args){
  uint32_t i = 0;
  char * s = NULL;
  char * newDest = NULL;
  char * temp = NULL;
  char tempBuffer[34];

  uint32_t index = 0;
  double q = 0.0;

  while(*fmt != '\0'){
    if (*fmt == '%'){
      ++fmt;
      switch(*fmt){
        case 'c' :
          i = va_arg(args,int);
          /*temp = stm32_itoa(i,&tempBuffer[33],10);*/
          buffer[index] = (char)i;
          ++index;
          break;
        case 'd' :
          i = va_arg(args,int);
          tempBuffer[33] = '\0';
          temp = stm32_itoa(i,&tempBuffer[33],10);
          newDest = stm32_strcpy(temp, &buffer[index]);
          index += (newDest-&buffer[index]);
          break;
        case 's' :
          s = va_arg(args,char *);
          newDest = stm32_strcpy(s, &buffer[index]);
          index += (newDest-&buffer[index]);
          break;
        case 'x':
          i = va_arg(args, int);
          tempBuffer[33] = '\0';
          temp = stm32_convertToHex(i,&tempBuffer[33],0);
          newDest = stm32_strcpy(temp, &buffer[index]);
          index += (newDest-&buffer[index]);
          break;
        case 'X':
          i = va_arg(args, int);
          tempBuffer[33] = '\0';
          temp = stm32_convertToHex(i,&tempBuffer[33],1);
          newDest = stm32_strcpy(temp, &buffer[index]);
          index += (newDest-&buffer[index]);
          break;
        case 'F':
        case 'f': /* Only supporting %.2f */
          q = va_arg(args, double);
          tempBuffer[33] = '\0';
          temp = stm32_floatToString(q, &tempBuffer[33]);
          newDest = stm32_strcpy(temp, &buffer[index]);
          index += (newDest-&buffer[index]);
          break;
      }
    }
    else{
      buffer[index] = *fmt;
      ++index;
    }
    ++fmt;
  }
  buffer[index] = '\0';

  return --index;
}

void stm32_printf(const char * fmt, ...){

  uint32_t num = 0;
  char buffer[100];
  va_list argp;
  va_start(argp,fmt);
  num = stm32_vsprintf(buffer,fmt,argp);
  va_end(argp);

  return;
}

