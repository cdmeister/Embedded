#include "stm32f407xx.h"
#include "stdlib.h"
#include "string.h"

/* Work out end of RAM address as intial stack pointer, Use both
 * (specific of a given STM32 MCU)
*/
#define SRAM_SIZE            128 * 1024 /* STM32F4O7 has 128 KB of RAM */
#define SRAM_END             (SRAM1_BASE + SRAM_SIZE)
#define RCC_AHB1ENR           ((uint32_t*)(RCC_BASE+ 0x30))



volatile uint32_t __attribute__((used)) dataVar = 0x3F;
volatile uint32_t __attribute__((used)) bssVar;
const uint32_t val = 1111;
const char msg[] = "Hello World";

void delay ( uint32_t count);


int main() {
  const uint32_t * p = &val;
  const uint32_t size_string = strlen(msg);
  char * heapMsg =(char *)malloc(sizeof(char)* size_string);
  strcpy(heapMsg,msg);
  /* Enable clock on GPIOA peripheral */
  *RCC_AHB1ENR |= (0x1<<0x3U);
  /* Configure the PA5 as output pull-up */
  GPIOD->MODER |= GPIO_MODER_MODE12_0; /* Sets MODER[11:10] = 0x1*/
  bssVar = 0x3F;
  free(heapMsg);
  while(bssVar == dataVar) {
    /* CORTEX M4F only support single precision hence the X.XXF */
    GPIOD->ODR ^= 0x1000;
    delay(200000);
    GPIOD->ODR ^= 0x0000;
    delay(200000);
  }
  for(;;);
  return 0;
}
void delay(uint32_t count) {
  if(count > 100){count +=val;}
  while(count--);
}
