#include "CS43L22.h"


void CS43L22_init(){

  // Setup All of the GPIO Pins for
  //  1. I2C1
  //  2. I2S3
  //  3. CS43L22 Audio Reset
  CS43L22_GPIO_init();


  // Setup the I2C1 module

  return;
}

static void CS43L22_GPIO_init(){

  // ---- Setup the GPIO Pins directly connect to CS43L22
  // ---- I2C: PB6 and PB9 are already pulled up so you don't need any
  // ---- internal pull up resistor. Configure it as output and open drain

  /* Enable Peripheral Clock for GPIOB */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);

 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOB->MODER &=~(GPIO_MODER_MODE6|GPIO_MODER_MODE9);
  GPIOB->MODER |=(GPIO_MODER_MODE6_1| GPIO_MODER_MODE9_1);

  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6|GPIO_AFRL_AFSEL9);
  GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL6_2|GPIO_AFRL_AFSEL9_2);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6|GPIO_OTYPER_OT9);
  GPIOB->OTYPER |=  (GPIO_OTYPER_OT6|GPIO_OTYPER_OT9);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOB->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED9);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6|GPIO_PUPDR_PUPD9);

  // ---- Setup the GPIO Pins directly connect to CS43L22
  // ---- PD4: Reset Pin

  /* Enable Peripheral Clock for GPIOD */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);

 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOD->MODER &=~(GPIO_MODER_MODE4);
  GPIOD->MODER |= (GPIO_MODER_MODE4_0);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT4);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED4);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD4);


  // ---- I2S GPIO Pin Setup

  /* Enable Peripheral Clock for GPIOC */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);

  // PC7: I2S3_MCK
  // PC10: I2S3_SCK
  // PC12: I2S3_SD
  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOC->MODER &=~(GPIO_MODER_MODE7|GPIO_MODER_MODE10|GPIO_MODER_MODE12);
  GPIOC->MODER |=(GPIO_MODER_MODE7_1| GPIO_MODER_MODE10_1|GPIO_MODER_MODE12_1);

  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL7);
  GPIOC->AFR[0] |= (GPIO_AFRL_AFSEL7_1|GPIO_AFRL_AFSEL7_2);

  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10|GPIO_AFRH_AFSEL12);
  GPIOC->AFR[1] |= (GPIO_AFRH_AFSEL10_1|GPIO_AFRH_AFSEL10_2|
                    GPIO_AFRH_AFSEL12_1|GPIO_AFRH_AFSEL12_2);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT7|GPIO_OTYPER_OT10|GPIO_OTYPER_OT12);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOC->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED7|GPIO_OSPEEDR_OSPEED10
                    |GPIO_OSPEEDR_OSPEED12);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD7|GPIO_PUPDR_PUPD10|GPIO_PUPDR_PUPD12);

  // PA4: I2S3_WS
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOA->MODER &=~( GPIO_MODER_MODE4);
  GPIOA->MODER |=( GPIO_MODER_MODE4_1); //Alternate Function

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL4_1|GPIO_AFRL_AFSEL4_2);


  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOA->OTYPER &= ~( GPIO_OTYPER_OT4);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOA->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED4); /* Configure as low speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPD4);

  return;
}

static void CS43L22_I2C_init(){

  // Enable I2C Periphal bus
  RCC->AHB1ENR |= (RCC_AHB1ENR_I2C1EN);

  // Disable I2C for configuration
  I2C->CR1 &=~(I2C_CR1_PE);


}


static void CS43L22_I2S_init(){


}
