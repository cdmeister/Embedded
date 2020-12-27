#include "debug_helper.h"

/* It is expected that param str has '\0' if needed.
 * this is to allow greater flexibility
 */
char * stm32_itoa(int num, char * str, uint32_t base)
{

  int i = 0;
  int isNegative = 0;
  if(num < 0) {
    isNegative = 1;
    num *=-1;
  }


  i = num;

  /* max number of digits would be base 2 at max value(2^31-1) plus 1 extra
   * for the negative sign
   */
  while(i>0){
    int remainder = i % base;
   /* printf("i %d \t remainder %d\n", i, remainder);*/
    --str;
    *str= remainder + '0';
    i=i/base;
  }

  if(isNegative == 1) {
    *str = '-';
  }
  /*printf("%s: Address of tempBuffer %p\n",__FUNCTION__ ,str);*/
  return str;
}

char * stm32_strcpy(char * pSrc, char * pDest){

  while(*pSrc){
    *pDest = *pSrc;
    ++pDest;
    ++pSrc;
  }
  return pDest;

}


char * stm32_convertToHex(uint32_t num, char * str, uint32_t alphaCase){

  char hexArray[22] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                       'A', 'B', 'C', 'D', 'E', 'F', 'a', 'b', 'c', 'd',
                       'e', 'f'};
  int shift = 4;
  while(num>0){
    int remainder = num % 16;
    --str;
    if(remainder < 10){
      *str = hexArray[remainder];
    }
    else{
      *str = (alphaCase==0) ? hexArray[remainder+6] : hexArray[remainder];
    }
    num >>=shift;
  }
  return str;
}


char * stm32_floatToString(float num, char * str){

  char * decpartStr = NULL;
  char * intpartStr = NULL;

  int intpart = (int)num;
  int decpart = (int)((num - intpart) * 100);/* The 100 multiplication controls number of dec */

  decpartStr = stm32_itoa(decpart, str, 10);
  str=decpartStr;
  --str;
  *str = '.';
  intpartStr = stm32_itoa(intpart, str, 10);
  str=intpartStr;

  return str;

}


