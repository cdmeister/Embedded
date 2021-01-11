#ifndef __HELPER_H
#define __HELPER_H

#include "stm32f407xx.h"

void rcc_gpio_enable(GPIO_TypeDef * GPIOx);
void rcc_adc_enable(ADC_TypeDef * ADCx);


#endif
