#include "helper.h"

void rcc_gpio_enable(GPIO_TypeDef * GPIOx){

  if(GPIOx == GPIOA)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
  if(GPIOx == GPIOB)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
  if(GPIOx == GPIOC)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);
  if(GPIOx == GPIOD)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);
  if(GPIOx == GPIOE)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOEEN);
  if(GPIOx == GPIOH)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOHEN);
  return;
}

void rcc_adc_enable(ADC_TypeDef * ADCx){

  /* Enable ADCx */
  if(ADCx == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  else if(ADCx == ADC2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
  else RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

  return;
}


