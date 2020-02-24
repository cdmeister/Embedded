/**
  * ST L6205 H-Bridge
  *
  */
#ifndef _L6205_H_
#define _L6205_H_

#include "stm32f407xx.h"
#include "helper.h"

uint32_t L6205_init(GPIO_TypeDef * GPIOx, TIM_TypeDef * TIMx);
uint32_t L6205_gpio_init(GPIO_TypeDef * GPIOx);
uint32_t L6205_timer_init(TIM_TypeDef * TIMx);



#endif /*_L6205_H_*/
