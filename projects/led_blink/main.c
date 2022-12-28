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
/*volatile uint32_t TimeDelay;*/
volatile uint32_t counter_toggle;
const uint32_t one_sec =100;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

  /*Enable Clock*/
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  /* Set mode of all pins as digital output */
  /* 00 = digital input         01 = digital output */
  /* 10 = alternate function    11 = analog (default) */
  GPIOD->MODER &=~(GPIO_MODER_MODE15|GPIO_MODER_MODE14
                  |GPIO_MODER_MODE12|GPIO_MODER_MODE13);
  GPIOD->MODER |= (GPIO_MODER_MODE15_0|GPIO_MODER_MODE14_0
                  |GPIO_MODER_MODE12_0|GPIO_MODER_MODE13_0);

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
   GPIOD->ODR ^=PORTD_12; 
   GPIOD->ODR ^=PORTD_13;
   GPIOD->ODR ^=PORTD_14; 
   GPIOD->ODR ^=PORTD_15; 
   Delay(1000);
  
  }
  return 0;
}
