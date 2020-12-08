#include "L6205.h"

uint32_t L6205_init(GPIO_TypeDef * GPIOx, TIM_TypeDef * TIMx){

  L6205_gpio_init(GPIOx);
  L6205_timer_init(TIMx);
  return 0;

}
uint32_t L6205_gpio_init(GPIO_TypeDef * GPIOx){

  /* Motor driver L205N */
  rcc_gpio_enable(GPIOx);
  /*   Set mode of all pins as digital output
   *   00 = digital input         01 = digital output
   *   10 = alternate function    11 = analog (default)
   */
  GPIOx->MODER &=~( GPIO_MODER_MODE7  | GPIO_MODER_MODE6
                  | GPIO_MODER_MODE5  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE1  | GPIO_MODER_MODE0);

  GPIOx->MODER |= ( GPIO_MODER_MODE7_0  |  GPIO_MODER_MODE6_0
                  | GPIO_MODER_MODE5_0  |  GPIO_MODER_MODE4_0);

  GPIOx->MODER |=(GPIO_MODER_MODE1_1| GPIO_MODER_MODE0_1);

 /* PWM Alternate Function */
  GPIOx->AFR[0] &= ~(GPIO_AFRL_AFSEL1|GPIO_AFRL_AFSEL0);
  GPIOx->AFR[0] |= (GPIO_AFRL_AFSEL1_1|GPIO_AFRL_AFSEL0_1);


  /*  Set output tupe of all pins as push-pull
   *  0 = push-pull (default)
   *  1 = open-drain
   */
  GPIOx->OTYPER &= ~( GPIO_OTYPER_OT7  | GPIO_OTYPER_OT6
                    | GPIO_OTYPER_OT5  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT0);

  /*  Set output speed of all pins as high
   *  00 = Low speed           01 = Medium speed
   *  10 = Fast speed          11 = High speed
   */
  GPIOx->OSPEEDR &=~(  GPIO_OSPEEDR_OSPEED7  | GPIO_OSPEEDR_OSPEED6
                    | GPIO_OSPEEDR_OSPEED4  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0);

  GPIOx->OSPEEDR |= (  GPIO_OSPEEDR_OSPEED7  | GPIO_OSPEEDR_OSPEED6
                    | GPIO_OSPEEDR_OSPEED4  | GPIO_OSPEEDR_OSPEED5
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0);


  /* Set all pins as no pull-up, no pull-down
   * 00 = no pull-up, no pull-down    01 = pull-up
   * 10 = pull-down,                  11 = reserved
   */
  GPIOx->PUPDR &= ~( GPIO_PUPDR_PUPD7  | GPIO_PUPDR_PUPD6 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD4  | GPIO_PUPDR_PUPD5
                  | GPIO_PUPDR_PUPD1  | GPIO_PUPDR_PUPD0);


  return 0;
}

uint32_t L6205_timer_init(TIM_TypeDef * TIMx){

  uint32_t TIMxCOUNTER_Frequency = 28e6; /*Desired Frequency*/

  /* Enable Timer 3 clock*/
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  /* Disable Timer 3*/
  TIMx->CR1 &= ~TIM_CR1_CEN;

  /* Counting Direction: 0 = up-counting, 1 = down-counting*/
  TIMx->CR1 &=~(TIM_CR1_DIR);

  /* Auto-reload preload enable */
  TIMx->CR1 &=~(TIM_CR1_ARPE);
  TIMx->CR1 |=(TIM_CR1_ARPE);

  /* Clock Prescaler */
  TIMx->PSC = (84000000/TIMxCOUNTER_Frequency)-1;

  /* Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0) */
  TIMx->ARR = 999;

  /* ------------------Channel 3 Setup ----------------------------------*/

  /* Disable Input/Output for Channel 3
  * This must be disable in order to set the channel as
  * Input or Output
  */
  TIMx->CCER &= ~TIM_CCER_CC3E;

  /* Set Channel 3 as output channel*/
  TIMx->CCMR2 &= ~(TIM_CCMR2_CC3S);

  /* Set the first value to compare against
  * 50% duty cycle
  */
  TIMx->CCR3=0*TIMx->ARR;

  /* Clear Output compare mode bits for channel 3
   * Select Pulse Width Modulation Mode 1
   */
  TIMx->CCMR2 |= (TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1);

  /* Select Preload Enable to be enable for PWM, allow to update CCR3 register
   * to be updated at overflow/underflow events
   */
  TIMx->CCMR2 &=~(TIM_CCMR2_OC3PE);
  TIMx->CCMR2 |= (TIM_CCMR2_OC3PE);

  /* Select Ouput polarity: 0 = active high, 1 = active low */
  TIMx->CCER &= ~(TIM_CCER_CC3P);

  /* Compare/Caputre output Complementary Polarity*/
  /* Must be kept at reset for channel if configured as output*/
  TIMx->CCER &=~(TIM_CCER_CC3NP);

  /* Enable Output for channel 3*/
  TIMx->CCER |= TIM_CCER_CC3E;


  /* ------------------Channel 4 Setup ----------------------------------*/

  /* Disable Input/Output for Channel 4
  * This must be disable in order to set the channel as
  * Input or Output
  */
  TIMx->CCER &= ~TIM_CCER_CC4E;

  /* Set Channel 4 as output channel*/
  TIMx->CCMR2 &= ~(TIM_CCMR2_CC4S);

  /* Set the first value to compare against
  * 50% duty cycle
  */
  TIMx->CCR4=0*TIMx->ARR;

  /* Clear Output compare mode bits for channel 4
   * Select Pulse Width Modulation Mode 1
   */
  TIMx->CCMR2 |= (TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1);

  /* Select Preload Enable to be enable for PWM, allow to update CCR4 register
   * to be updated at overflow/underflow events
   */
  TIMx->CCMR2 &=~(TIM_CCMR2_OC4PE);
  TIMx->CCMR2 |= (TIM_CCMR2_OC4PE);

  /* Select Ouput polarity: 0 = active high, 1 = active low */
  TIMx->CCER &= ~(TIM_CCER_CC4P);

  /* Compare/Caputre output Complementary Polarity
   * Must be kept at reset for channel if configured as output
   */
  TIMx->CCER &=~(TIM_CCER_CC4NP);

  /* Enable Output for channel 4 */
  TIMx->CCER |= TIM_CCER_CC4E;

  /* --------------------------------------------------------------*/

  /* Enable Update Generation*/
  TIMx->EGR &= ~TIM_EGR_UG;
  TIMx->EGR |= TIM_EGR_UG;

  /* Center Align Mode Selection*/
  TIMx->CR1 &=~(TIM_CR1_CMS);

  /*Disable interrupts only on channel 4*/
  /*TIMx->DIER &=~(TIM_DIER_CC4IE);*/

  /* Enable Timer 4 after all of the initialization*/
  TIMx->CR1 |= TIM_CR1_CEN;

  return 0;
}
