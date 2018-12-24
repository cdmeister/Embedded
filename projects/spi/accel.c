/**
  ******************************************************************************
  * @file    main.c
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    9-September-2017
  * @brief   Main program body
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

#define PORTD_15 0x00008000
#define PORTD_14 0x00004000
#define PORTD_13 0x00002000
#define PORTD_12 0x00001000
#define PORTD_ALL 0x0000F000

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /* LED Init */
  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOD->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13
                   | GPIO_MODER_MODE14 | GPIO_MODER_MODE15); /* Clear mode bits */
  GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0
                   | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);/* LED 5-8 are on GPIOD Pins 12-15 */

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 |
                   GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15); /*Configure as output open-drain */

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | /* Configure as high speed */
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13
                  | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15); /*no pul-up, no pull-down*/




  /* Pin setup for SPI command */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN);
  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7); /* Clear mode bits */
  GPIOA->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); /* Alternate Function */

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL6_0 |
                    GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_1);

  GPIOE->MODER &= ~(GPIO_MODER_MODE3);
  GPIOE->MODER |= (GPIO_MODER_MODE3_0);

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7); /*Configure as output open-drain */
  GPIOE->OTYPER &= ~(GPIO_OTYPER_OT3);
  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOA->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 | /* Configure as high speed */
                    GPIO_OSPEEDR_OSPEED7); /* Configure as high speed */
  GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 |
                    GPIO_OSPEEDR_OSPEED7);
  GPIOE->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED3); /* Configure as high speed */
  GPIOE->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7); /*no pul-up, no pull-down*/
  GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD3); /*no pul-up, no pull-down*/


  // Generate interupt every 1ms
  // Since clock is 168 MHz, then 168MHz/1000 is
  // 168000 clock ticks inorder to generate an interupt every 1ms
  SysTick_Init(SystemCoreClock/1000);

  /* Infinite loop */
  while (1)
  {
    GPIOD->ODR |=PORTD_12;
    Delay(1000);
    GPIOD->ODR &=~PORTD_12;
    Delay(1000);
    GPIOD->ODR |=PORTD_13;
    Delay(1000);
    GPIOD->ODR &=~PORTD_13;
    Delay(1000);
    GPIOD->ODR |=PORTD_14;
    Delay(1000);
    GPIOD->ODR &=~PORTD_14;
    Delay(1000);
    GPIOD->ODR |=PORTD_15;
    Delay(1000);
    GPIOD->ODR &=~PORTD_15;
    Delay(1000);
    GPIOD->ODR |=PORTD_ALL;

    Delay(1000);
    GPIOD->ODR &=~PORTD_ALL;
    Delay(1000);


  }

  return 0;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
