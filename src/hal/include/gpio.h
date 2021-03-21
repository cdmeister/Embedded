#ifndef GPIO_H
#define GPIO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
#include "error.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void GPIO_ClockInit(GPIO_TypeDef * GPIOx);
void GPIO_Mode(GPIO_TypeDef * GPIOx, uint32_t ModeMask, uint32_t value);
void GPIO_OTyper(GPIO_TypeDef * GPIOx, uint32_t OtyperMask, uint32_t value);
void GPIO_OSpeedr(GPIO_TypeDef * GPIOx, uint32_t OspeedrMask, uint32_t value);
void GPIO_Pupdr(GPIO_TypeDef * GPIOx, uint32_t PupdrMask, uint32_t value);

#endif /* GPIO_H */
