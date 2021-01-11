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
#include "startup.h"
#include "stm32f407xx.h"
#include "lcd.h"
#include "gpio.h"
#include "pushbutton.h"
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
        startup.c file
     */
  LCD rgb_lcd;
  SysTick_Init(SystemCoreClock/1000);
  GPIO_ClockInit(GPIOD);
  GPIO_ClockInit(GPIOC);
  GPIO_Mode(GPIOD,
                  /* Mask */
                  ( GPIO_MODER_MODE15 | GPIO_MODER_MODE14
                  | GPIO_MODER_MODE13 | GPIO_MODER_MODE12
                  | GPIO_MODER_MODE10 | GPIO_MODER_MODE9
                  | GPIO_MODER_MODE8  | GPIO_MODER_MODE7
                  | GPIO_MODER_MODE6  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3  | GPIO_MODER_MODE2
                  | GPIO_MODER_MODE1  | GPIO_MODER_MODE0),
                  /* Value */
                  ( GPIO_MODER_MODE15_0 |  GPIO_MODER_MODE14_0
                  | GPIO_MODER_MODE13_0 |  GPIO_MODER_MODE12_0
                  | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE9_0
                  | GPIO_MODER_MODE8_0  | GPIO_MODER_MODE7_0
                  | GPIO_MODER_MODE6_0  |  GPIO_MODER_MODE4_0
                  | GPIO_MODER_MODE3_0  |  GPIO_MODER_MODE2_0
									| GPIO_MODER_MODE1_0 | GPIO_MODER_MODE0_0));

  GPIO_Mode(GPIOC, GPIO_MODER_MODE1, (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1));

  GPIO_OTyper(GPIOD,/* Mask */
                    ( GPIO_OTYPER_OT15 | GPIO_OTYPER_OT14
                    | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT12
                    | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT9
                    | GPIO_OTYPER_OT8  | GPIO_OTYPER_OT7
                    | GPIO_OTYPER_OT6  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3  | GPIO_OTYPER_OT2
                    | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT0),
                    /* Value */
                    ~( GPIO_OTYPER_OT15 | GPIO_OTYPER_OT14
                    | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT12
                    | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT9
                    | GPIO_OTYPER_OT8  | GPIO_OTYPER_OT7
                    | GPIO_OTYPER_OT6  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3  | GPIO_OTYPER_OT2
                    | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT0));


  GPIO_OTyper(GPIOC, GPIO_OTYPER_OT1, ~(GPIO_OTYPER_OT1));

  GPIO_OSpeedr(GPIOD,/* Mask */
                    ( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9
                    | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED7
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0),
                    /* Value */
                    ( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9
                    | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED7
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0)); /* Configure as high speed */


  GPIO_OSpeedr(GPIOC, GPIO_OSPEEDR_OSPEED1, GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */

  GPIO_Pupdr(GPIOD, /* Mask */
                  (GPIO_PUPDR_PUPD15 | GPIO_PUPDR_PUPD14
                  | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD12
                  | GPIO_PUPDR_PUPD10  | GPIO_PUPDR_PUPD9 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD8  | GPIO_PUPDR_PUPD7 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD6  | GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD3  | GPIO_PUPDR_PUPD2
                  | GPIO_PUPDR_PUPD1  | GPIO_PUPDR_PUPD0),
                  /* Value */
                  ~(GPIO_PUPDR_PUPD15 | GPIO_PUPDR_PUPD14
                  | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD12
                  | GPIO_PUPDR_PUPD10  | GPIO_PUPDR_PUPD9 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD8  | GPIO_PUPDR_PUPD7 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD6  | GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD3  | GPIO_PUPDR_PUPD2
                  | GPIO_PUPDR_PUPD1  | GPIO_PUPDR_PUPD0));

  GPIO_Pupdr(GPIOC,GPIO_PUPDR_PUPD1, ~(GPIO_PUPDR_PUPD1));


  LCD_init(&rgb_lcd,GPIOD,0,0,1,2,3,4,6,7,8,9,10,4,20,LCD_8BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  GPIOD->ODR ^=PORTD_12;
    /*GPIOD->ODR ^=PORTD_13; */
   /* GPIOD->ODR ^=PORTD_14; */
   /* GPIOD->ODR ^=PORTD_15; */
    Delay(500);
    GPIOD->ODR ^=PORTD_13;

  LCD_print(&rgb_lcd, "Periphery Setup Complete %d", SystemCoreClock);
  Delay(500);
  GPIOD->ODR ^=PORTD_14;
  Delay(1000);

  LCD_clear(&rgb_lcd);
  /*LCD_home(&rgb_lcd);*/
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);


  /*unsigned int delay =0;*/

  /* Enable Clock */
  /*RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;*/


  /* Set mode of all pins as digital output
   *  00 = digital input       01 = digital output
   *  10 = alternate function  11 = analog (default)
   */
  /*GPIOD->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13
                   | GPIO_MODER_MODE14 | GPIO_MODER_MODE15);*/ /* Clear mode bits */
  /*GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0
                   | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);*//* LED 5-8 are on GPIOD Pins 12-15 */

  /* Set output type of all pins as push-pull
   * 0 = push-pull (default)
   * 1 = open-drain
   */
  /*GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 |
                   GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15);*/ /*Configure as output open-drain */

  /* Set output speed of all pins as high
   * 00 = Low speed           01 = Medium speed
   * 10 = Fast speed          11 = High speed
   */
  /*GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |*/ /* Configure as high speed */
  /*                  GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);*/ /* Configure as high speed */
  /*GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);*/

  /* Set all pins as no pull-up, no pull-down
   * 00 = no pull-up, no pull-down    01 = pull-up
   * 10 = pull-down,                  11 = reserved
   */
  /*GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13
                  | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15);*/ /*no pul-up, no pull-down*/

  while (1)
  {

    LCD_print(&rgb_lcd, "ADC TEMP: ");
    LCD_setCursor(&rgb_lcd, 0,1);
  }
  return 0;
}
