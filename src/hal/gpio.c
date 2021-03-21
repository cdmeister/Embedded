#include "gpio.h"

void GPIO_ClockInit(GPIO_TypeDef * GPIOx){

  if(GPIOx == GPIOA)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
  else if(GPIOx == GPIOB)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
  else if(GPIOx == GPIOC)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);
  else if(GPIOx == GPIOD)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);
  else if(GPIOx == GPIOE)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOEEN);
  else if(GPIOx == GPIOH)  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOHEN);
  else{
    Default_Handler();
  }
  return;

}

void GPIO_Mode(GPIO_TypeDef * GPIOx, uint32_t ModeMask, uint32_t value){
  /* Reset Pins first */
  GPIOx->MODER &= (~ModeMask);

  /* Set the value */
  GPIOx->MODER |= value;
}

void GPIO_OTyper(GPIO_TypeDef * GPIOx, uint32_t OTyperMask, uint32_t value){
  /* Reset Pins first */
  GPIOx->OTYPER &= (~OTyperMask);

  /* Set the value */
  GPIOx->OTYPER |= value;

}

void GPIO_OSpeedr(GPIO_TypeDef * GPIOx, uint32_t OSpeedrMask, uint32_t value){

   /* Reset Pins first */
  GPIOx->OSPEEDR &= (~OSpeedrMask);

  /* Set the value */
  GPIOx->OSPEEDR |= value;


}

void GPIO_Pupdr(GPIO_TypeDef * GPIOx, uint32_t PupdrMask, uint32_t value){

  /* Reset Pins first */
  GPIOx->PUPDR &= (~PupdrMask);

  /* Set the value */
  GPIOx->PUPDR |= value;


}





