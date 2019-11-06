#include "stdlib.h"
#include "string.h"
#include "linker_defines.h"



extern void main(void);



void _initialize_data(uint32_t * flash_begin, uint32_t * data_begin,
                              uint32_t * data_end){
  uint32_t * p = data_begin;
  while(p<data_end) *p++ = *flash_begin++;

}

void _initialize_bss(uint32_t * bss_begin, uint32_t * bss_end){
  uint32_t * p = bss_begin;
  while(p<bss_end) *p++ = 0;

}

void Reset_Handler(void){
  _initialize_data(&_sidata,&_sdata,&_edata);
  _initialize_bss(&_sbss,&_ebss);
  main();
  for(;;);
}



