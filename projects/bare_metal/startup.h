#ifndef __STARTUP_H
#define __STARTUP_H

#include "stdlib.h"
#include "string.h"
#include "linker_defines.h"
#include "stm32f407.h"

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */

void _initialize_data(uint32_t * flash_begin, uint32_t * data_begin,
                      uint32_t * data_end);

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */

void _initialize_bss(uint32_t * bss_begin, uint32_t * bss_end);

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void);

#endif /* __STARTUP_H */
