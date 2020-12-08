#ifndef __PUSHBUTTON_H
#define __PUSHBUTTON_H

#include "stm32f407xx.h"
#include "helper.h"

uint16_t getADCx_coordinate(void);
uint16_t getADCy_coordinate(void);
uint32_t pushbutton_adc_init(ADC_TypeDef * ADCx);
uint32_t pushbutton_gpio_init(GPIO_TypeDef * GPIOx);
uint32_t pushbutton_init(GPIO_TypeDef * GPIOx, ADC_TypeDef * ADCx);

#endif
