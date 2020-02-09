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
#include "stm32f407xx.h"



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
  unsigned int delay =0;

  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;


  /* Set mode of all pins as digital output
   *  00 = digital input       01 = digital output
   *  10 = alternate function  11 = analog (default)
   */
  GPIOD->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13
                   | GPIO_MODER_MODE14 | GPIO_MODER_MODE15); /* Clear mode bits */
  GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0
                   | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);/* LED 5-8 are on GPIOD Pins 12-15 */

  /* Set output type of all pins as push-pull
   * 0 = push-pull (default)
   * 1 = open-drain
   */
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 |
                   GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15); /*Configure as output open-drain */

  /* Set output speed of all pins as high
   * 00 = Low speed           01 = Medium speed
   * 10 = Fast speed          11 = High speed
   */
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | /* Configure as high speed */
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);

  /* Set all pins as no pull-up, no pull-down
   * 00 = no pull-up, no pull-down    01 = pull-up
   * 10 = pull-down,                  11 = reserved
   */
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13
                  | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15); /*no pul-up, no pull-down*/


  /* Infinite loop */
  while (1)
  {
    GPIOD->ODR |=PORTD_12;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_12;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR |=PORTD_13;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_13;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR |=PORTD_14;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_14;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR |=PORTD_15;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_15;

    for(delay= 0; delay < 1066667; delay++);

    GPIOD->ODR |=PORTD_ALL;

    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_ALL;
    for(delay= 0; delay < 1066667; delay++);

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
