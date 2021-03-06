/**
  ******************************************************************************
  * @file    main.c
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    17-September-2017
  * @brief   Main program body
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "startup.h"
#include "systick.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORTD_12 0x00001000 /*Green */
#define PORTD_13 0x00002000 /*Orange */
#define PORTD_14 0x00004000 /*Red */
#define PORTD_15 0x00008000 /*Blue */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimeDelay;
volatile uint32_t counter_toggle;
const uint32_t one_sec =100;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TIM4_IRQHandler(void){

  /* Check whether an overflow event has taken place */
  if((TIM4->SR & TIM_SR_CC2IF) != 0){
    if(counter_toggle == one_sec){
      GPIOD->ODR ^=(PORTD_12);
      counter_toggle =0;
    }
    else{
      counter_toggle++;
    }
    TIM4->SR &= ~TIM_SR_CC2IF;
  }
  /*if((TIM4->SR & TIM_SR_CC4IF) != 0){
    GPIOD->ODR ^=(PORTD_13);
    TIM4->SR &= ~TIM_SR_CC4IF;
  }*/
  /* Check whether an overflow event has taken place */
  /*if((TIM4->SR & TIM_SR_UIF) != 0){

    TIM4->SR &= ~TIM_SR_UIF;

  }*/

}
uint32_t timer_init(void){

  uint32_t TIM4COUNTER_Frequency = 50; /*Desired Frequency */

  /* Enable Timer 4 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  /* Disable Timer 4 */
  TIM4->CR1 &= ~TIM_CR1_CEN;

  /* Counting Direction: 0 = up-counting, 1 = down-counting */
  TIM4->CR1 &=~(TIM_CR1_DIR);

  /* Auto-reload preload enable */
  TIM4->CR1 &=~(TIM_CR1_ARPE);
  TIM4->CR1 |=(TIM_CR1_ARPE);

  /*Clock Prescaler */
  TIM4->PSC = 83;
  /*TIM4->PSC = 62499; */

  /*Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0) */
  TIM4->ARR = 19999;
  /*TIM4->ARR = 1343; */

  /* ------------------Channel 2 Setup ---------------------------------- */

  /* Disable Input/Output for Channel 3 */
  /* This must be disable in order to set the channel as */
  /* Input or Output */
  TIM4->CCER &= ~TIM_CCER_CC2E;

  /* Set Channel 4 as output channel */
  TIM4->CCMR1 &= ~(TIM_CCMR1_CC2S);

  /* Set the first value to compare against */
  /* 50% duty cycle */
  /*TIM4->CCR3=.5*TIM4->ARR; */
  TIM4->CCR2 = TIM4->ARR;

  /* Clear Output compare mode bits for channel 3 */
  TIM4->CCMR1 &= ~TIM_CCMR1_OC2M;

  /* Select Pulse Width Modulation Mode 1 */
  TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1);

  /* Select Preload Enable to be enable for PWM, allow to update CCR4 register */
  /* to be updated at overflow/underflow events */
  TIM4->CCMR1 &=~(TIM_CCMR1_OC2PE);

  /* Select Ouput polarity: 0 = active high, 1 = active low */
  TIM4->CCER &= ~(TIM_CCER_CC2P);

  /* Compare/Caputre output Complementary Polarity */
  /* Must be kept at reset for channel if configured as output */
  TIM4->CCER &=~(TIM_CCER_CC2NP);

  /* Enable Output for channel 4 */
  TIM4->CCER |= TIM_CCER_CC2E;


  /* ------------------Channel 3 Setup ---------------------------------- */

  /* Disable Input/Output for Channel 3 */
  /* This must be disable in order to set the channel as */
  /* Input or Output */
  TIM4->CCER &= ~TIM_CCER_CC3E;

  /* Set Channel 4 as output channel */
  TIM4->CCMR2 &= ~(TIM_CCMR2_CC3S);

  /* Set the first value to compare against */
  /* 50% duty cycle */
  /*TIM4->CCR3=.5*TIM4->ARR; */
  TIM4->CCR3 = 0;

  /* Clear Output compare mode bits for channel 3 */
  TIM4->CCMR2 &= ~TIM_CCMR2_OC3M;

  /* Select Pulse Width Modulation Mode 1 */
  TIM4->CCMR2 |= (TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1);

  /* Select Preload Enable to be enable for PWM, allow to update CCR4 register */
  /* to be updated at overflow/underflow events */
  TIM4->CCMR2 &=~(TIM_CCMR2_OC3PE);
  TIM4->CCMR2 |= (TIM_CCMR2_OC3PE);

  /* Select Ouput polarity: 0 = active high, 1 = active low */
  TIM4->CCER &= ~(TIM_CCER_CC3P);

  /* Compare/Caputre output Complementary Polarity */
  /* Must be kept at reset for channel if configured as output */
  TIM4->CCER &=~(TIM_CCER_CC3NP);

  /* Enable Output for channel 4 */
  TIM4->CCER |= TIM_CCER_CC3E;

  /* ------------------Channel 4 Setup ---------------------------------- */

  /* Disable Input/Output for Channel 4 */
  /* This must be disable in order to set the channel as */
  /* Input or Output */
  TIM4->CCER &= ~TIM_CCER_CC4E;

  /* Set Channel 4 as output channel */
  TIM4->CCMR2 &= ~(TIM_CCMR2_CC4S);

  /* Set the first value to compare against */
  TIM4->CCR4=0;

  /* Clear Output compare mode bits for channel 1 */
  TIM4->CCMR2 &= ~TIM_CCMR2_OC4M;

  /* Select Pulse Width Modulation Mode 1 */
  TIM4->CCMR2 |= (TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1);

  /* Select Preload Enable to be enable for PWM, allow to update CCR4 register */
  /* to be updated at overflow/underflow events */
  TIM4->CCMR2 &=~(TIM_CCMR2_OC4PE);
  TIM4->CCMR2 |= (TIM_CCMR2_OC4PE);

  /* Select Ouput polarity: 0 = active high, 1 = active low */
  TIM4->CCER &= ~(TIM_CCER_CC4P);

  /* Compare/Caputre output Complementary Polarity */
  /* Must be kept at reset for channel if configured as output */
  TIM4->CCER &=~(TIM_CCER_CC4NP);

  /* Enable Output for channel 4 */
  TIM4->CCER |= TIM_CCER_CC4E;

  /* -------------------------------------------------------------- */

  /* Enable Update Generation */
  TIM4->EGR &= ~TIM_EGR_UG;
  TIM4->EGR |= TIM_EGR_UG;

  /* Center Align Mode Selection */
  TIM4->CR1 &=~(TIM_CR1_CMS);

  /*Clear interrupt status only on channel 3 and 4 */

  TIM4->SR &= ~(TIM_SR_CC2IF);

  /*Enable interrupts only on channel 3 and 4 */

  TIM4->DIER |= (TIM_DIER_CC2IE);


  /* Set TIM4 priority to 1 */
  NVIC_SetPriority(TIM4_IRQn,3);

  /* Enable TIM4 interrupt */
  NVIC_EnableIRQ(TIM4_IRQn);


  /* Enable Timer 4 after all of the initialization */
  TIM4->CR1 |= TIM_CR1_CEN;
  return TIM4->ARR;

}

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  const int max_brightness = timer_init();
  /*Enable Clock*/
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  /* Set mode of all pins as digital output */
  /* 00 = digital input         01 = digital output */
  /* 10 = alternate function    11 = analog (default) */
  GPIOD->MODER &=~(GPIO_MODER_MODE15|GPIO_MODER_MODE14
                  |GPIO_MODER_MODE12|GPIO_MODER_MODE13);
  GPIOD->MODER |=(GPIO_MODER_MODE13_0|GPIO_MODER_MODE12_0);
  /*                GPIO_MODER_MODE12_0|GPIO_MODER_MODE13_0); //output */
  GPIOD->MODER |=(GPIO_MODER_MODE15_1 | GPIO_MODER_MODE14_1);
                 /* GPIO_MODER_MODE13_1 | GPIO_MODER_MODE12_1); //alternate function */

  GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL15|GPIO_AFRH_AFSEL14);
  GPIOD->AFR[1] |= (GPIO_AFRH_AFSEL15_1|GPIO_AFRH_AFSEL14_1);

  /* Set output tupe of all pins as push-pull */
  /* 0 = push-pull (default) */
  /* 1 = open-drain */
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT15|GPIO_OTYPER_OT14
                    |GPIO_OTYPER_OT13|GPIO_OTYPER_OT12);

  /* Set output speed of all pins as high */
  /* 00 = Low speed           01 = Medium speed */
  /* 10 = Fast speed          11 = High speed */
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED15|GPIO_OSPEEDR_OSPEED14
                    |GPIO_OSPEEDR_OSPEED13|GPIO_OSPEEDR_OSPEED12); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED15|GPIO_OSPEEDR_OSPEED14
                    |GPIO_OSPEEDR_OSPEED13|GPIO_OSPEEDR_OSPEED12); /* Configure as high speed */

  /* Set all pins as no pull-up, no pull-down */
  /* 00 = no pull-up, no pull-down    01 = pull-up */
  /* 10 = pull-down,                  11 = reserved */
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD15|GPIO_PUPDR_PUPD14 /*no pul-up, no pull-down*/
                    |GPIO_PUPDR_PUPD13|GPIO_PUPDR_PUPD12);

  /* Generate and interrupt every 1ms */
  /* http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html */
  /* If the clock is at 168MHz, then that is 168 000 000 ticks per second */
  /* but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead */
  /* you can generate an interupt every 1ms so that would be 168 000 ticks per */
  /* ms and you can fit 168 000 ticks into the LOAD register */
  SysTick_Init(SystemCoreClock/1000);


  while(1){
    /*GPIOD->ODR ^=PORTD_12; */
    /*GPIOD->ODR ^=PORTD_13; */
   /* GPIOD->ODR ^=PORTD_14; */
   /* GPIOD->ODR ^=PORTD_15; */
    /*Delay(500); */
    TIM4->CCR4=900;
    Delay(1500);
    TIM4->CCR4=2100;
    Delay(1500);

  }
  return 0;
}
