#include "stdlib.h"
#include "string.h"
#include "stdint.h"

/* Memory and peripheral start address(common to all STM32 MCUs) */
#define FLASH_BASE            0x08000000U /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000U /*!< SRAM1(112 KB) base address in the alias region                              */
#define SRAM2_BASE            0x2001C000U /*!< SRAM2(16 KB) base address in the alias region                              */
#define PERIPH_BASE           0x40000000U /*!< Peripheral base address in the alias region                                */

/* Work out end of RAM address as intial stack pointer, Use both
 * (specific of a given STM32 MCU)
*/
#define SRAM_SIZE            128 * 1024 /* STM32F4O7 has 128 KB of RAM */
#define SRAM_END             (SRAM1_BASE + SRAM_SIZE)

/* Peripheral memory map */

#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000U)



/* RCC peripher address applicable to GPIOA
 * (specific of a given STM32 MCU)
*/
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800U)
#define RCC_AHB1ENR           ((uint32_t*)(RCC_BASE+ 0x30))



/* GPIOD peripheral address
 * (specific of a given STM32 MCU)
*/
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOD_MODER           ((uint32_t*)(GPIOD_BASE + 0x00))
#define GPIOD_ODR             ((uint32_t*)(GPIOD_BASE + 0x14))
#define GPIO_MODER_MODE12_Pos            (24U)
#define GPIO_MODER_MODE12_Msk            (0x3U << GPIO_MODER_MODE12_Pos)       /*!< 0x03000000 */
#define GPIO_MODER_MODE12                GPIO_MODER_MODE12_Msk
#define GPIO_MODER_MODE12_0              (0x1U << GPIO_MODER_MODE12_Pos)       /*!< 0x01000000 */

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
  *GPIOD_MODER |= GPIO_MODER_MODE12_0; /* Sets MODER[11:10] = 0x1*/
  bssVar = 0x3F;
  free(heapMsg);
  while(bssVar == dataVar) {
    /* CORTEX M4F only support single precision hence the X.XXF */
    *GPIOD_ODR ^= 0x1000;
    delay(200000);
    *GPIOD_ODR ^= 0x0000;
    delay(200000);
  }
  for(;;);
  return 0;
}
void delay(uint32_t count) {
  if(count > 100){count +=val;}
  while(count--);
}