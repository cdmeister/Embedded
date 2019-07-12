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
#include "lcd.h"
#include "helper.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NUM_SAMPLES 10
#define PORTD_15 0x00008000
#define PORTD_14 0x00004000
#define PORTD_13 0x00002000
#define PORTD_12 0x00001000
#define PORTD_ALL 0x0000F000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t counter =0;
uint16_t sample_array[10];
volatile uint16_t pot_adc1 = 0;
volatile uint16_t pot_adc2 = 0;
volatile uint8_t ready = 0;
LCD rgb_lcd;
RangeMap motor_foward, motor_backward;
/* Private function prototypes -----------------------------------------------*/
uint16_t motor_pwm(RangeMap * motor_foward,RangeMap * motor_backward, double input);
uint16_t motor_pwm_rl(double input);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

inline uint16_t motor_pwm(RangeMap * motor_foward, RangeMap * motor_backward, double input){

  if( input<2148 && input>1947){

    GPIOA->BSRR = (1<<23);
    GPIOA->BSRR = (1<<21);
    TIM3->CCR3=0;
    TIM3->CCR4=0;
    return 0;
  }
  else if ( input > 2147 && input <4096){
    uint16_t temp= 999 -(uint16_t) RangeMap_map(motor_foward,input);
GPIOA->BSRR = (3<<23);
    GPIOA->BSRR = (3<<21);

    GPIOA->BSRR = (1<<7);
    GPIOA->BSRR = (1<<5);
    TIM3->CCR3=temp;
    TIM3->CCR4=temp;
    return temp;
  }
  else{
    uint16_t temp = (uint16_t) RangeMap_map(motor_backward,input);
    GPIOA->BSRR = (3<<23);
    GPIOA->BSRR = (3<<21);
    TIM3->CCR3=temp;
    TIM3->CCR4=temp;
    return temp;
  }
}

inline uint16_t motor_pwm_rl(double input){

  if(input < 1948){
    TIM3->CCR3=750;


    return 1;
  }
  else if(input > 2147){
    TIM3->CCR4=750;


    return 2;
  }
  else{ return 3;}
}


void ADC_IRQHandler(void){
  if((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC){
    /* acknowledge interrupt */
    uint16_t value;
    value = ADC1->DR;
        if(counter % 2 == 0) {
                pot_adc1 = value;
        } else {
                pot_adc2 = value;
        }
        counter++;
  }

}

void EXTI0_IRQHandler(void){

  // Check for EXTI 0 flag
  if((EXTI->PR & EXTI_PR_PR0) == EXTI_PR_PR0){

    // Toggle all LED on board
    GPIOD->ODR ^= PORTD_ALL;

    // Clear interupt pending request
    EXTI->PR = EXTI_PR_PR0;
  }
}


void EXTI_Init2(void){

// Enable SYSCFG clock
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

// Select PC11 as the trigger source of EXTI 11
SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI11; //clear EXTI 11
SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PC; // set EXTI11 to map ext interrupt
                                               //  to PC11
// Enable rising edge trigger for EXTI 11
// Rising edge trigger selection register (RSTR)
// 0 = disable  1 = enable
EXTI->RTSR &= ~EXTI_RTSR_TR11;
EXTI->RTSR |= EXTI_RTSR_TR11;

// Disable Falling edge trigger EXTI 11
// Falling trigger selection
// 0 = disable  1 = enable
EXTI->FTSR &= ~EXTI_FTSR_TR11;

// Enable EXTI 11 Interrupt
// Interrupt mask register: 0 = masked, 1 = unmasked
// "Masked" means that processor ignores corresponding interupt
EXTI->IMR &= ~EXTI_IMR_MR11;
EXTI->IMR |= EXTI_IMR_MR11;


// Set EXTI 11 priority to 1
NVIC_SetPriority(EXTI15_10_IRQn, 2);

// Enable EXT 11 interrupt
NVIC_EnableIRQ(EXTI15_10_IRQn);


}

void EXTI15_10_IRQHandler(void){

  // Check for EXTI 11 flag
  if((EXTI->PR & EXTI_PR_PR11_Msk) == EXTI_PR_PR11){

    // Toggle all LED on board
    GPIOD->ODR ^= PORTD_ALL;

    // Clear interupt pending request
    EXTI->PR |= EXTI_PR_PR11;
  }
}


void EXTI_Init(void){

  // Enable SYSCFG clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Select PB0 as the trigger source of EXTI0
  SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; //clear EXTI 0
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // set EXTI0 to map ext interrupt
                                               //  to PB0
  // Enable rising edge trigger for EXTI0
  // Rising edge trigger selection register (RSTR)
  // 0 = disable  1 = enable
  EXTI->RTSR &= ~EXTI_RTSR_TR0;
  EXTI->RTSR |= EXTI_RTSR_TR0;

  // Disable Falling edge trigger EXTI0
  // Falling trigger selection
  // 0 = disable  1 = enable
  EXTI->FTSR &= ~EXTI_FTSR_TR0;

  // Enable EXTI0 Interrupt
  // Interrupt mask register: 0 = masked, 1 = unmasked
  // "Masked" means that processor ignores corresponding interupt
  EXTI->IMR &= ~EXTI_IMR_MR0;
  EXTI->IMR |= EXTI_IMR_MR0;


  // Set EXTI 0 priority to 1
  NVIC_SetPriority(EXTI0_IRQn, 2);

  // Enable EXT 0 interrupt
  NVIC_EnableIRQ(EXTI0_IRQn);


}

void ADCx_Init(ADC_TypeDef * ADCx){

  // Enable ADCx
  if(ADCx == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  else if(ADCx == ADC2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
  else RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;


  /*
   * ADC Mode Selection
   *
   * Note:
   *  00000 : Independent Mode, ADC operate independently
   */
  ADC123_COMMON->CCR &= ~(ADC_CCR_MULTI);


  /*
   * Set and cleared by software to select the frequency of the clock
   *  to the ADC. The clock is common for all the ADCs.
   *
   * Note:
   *  00: PCLK2 divided by 2
   *  01: PCLK2 divided by 4
   *  10: PCLK2 divided by 6
   *  11: PCLK2 divided by 8
  */
  ADC123_COMMON->CCR &= ~(ADC_CCR_ADCPRE);  // Clear
  ADC123_COMMON->CCR |= (ADC_CCR_ADCPRE_0); // DIV4


  // Disable DMA
  ADC123_COMMON->CCR &= ~(ADC_CCR_DMA);

  //Configurable delay between conversions in Dual/Triple interleaved mode
  ADC123_COMMON->CCR &= ~(ADC_CCR_DELAY);

  // Resolution ot 12-bits
  ADCx->CR1 &= ~(ADC_CR1_RES);

  // Scan Mode for this example
  ADCx->CR1 &=~( ADC_CR1_SCAN);
  ADCx->CR1 |=( ADC_CR1_SCAN);

  // Enable Continuos Mode
  ADCx->CR2 &= ~(ADC_CR2_CONT);

  // External Trigger on rising edge
  ADCx->CR2 &= ~(ADC_CR2_EXTEN);

  // No Timer Trigger to drive ADC conversion
  ADCx->CR2 &= ~(ADC_CR2_EXTSEL);

  // Data Alignment
  ADCx->CR2 &= ~(ADC_CR2_ALIGN);

  // Number of Conversions
  ADCx->SQR1 &= ~(ADC_SQR1_L);
  ADCx->SQR1 |= (ADC_SQR1_L_0); // 2 conversion

  // Disable Temperature/Vref
  ADC123_COMMON->CCR &=~(ADC_CCR_TSVREFE);


  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  // PC1 is connected to ADC channel 11
  ADCx->SQR3 |= (ADC_SQR3_SQ1_3|ADC_SQR3_SQ1_1|ADC_SQR3_SQ1_0);
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= (ADC_SMPR1_SMP11);

  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ2);
  // PC2 is connected to ADC channel 12
  ADCx->SQR3 |= (ADC_SQR3_SQ2_3|ADC_SQR3_SQ2_2);
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= (ADC_SMPR1_SMP12);


  // This call enables the end-of-conversion flag after each channel,
  // which triggers the end-of-conversion interrupt every time this flag is set.
  ADCx->CR2 |= ADC_CR2_EOCS;

  // Enable Regular channel Interrupt
  ADCx->CR1 |= ADC_CR1_EOCIE;


  // Set ADCx priority to 1
  NVIC_SetPriority(ADC_IRQn,1);

  // Enable ADCx interrupt
  NVIC_EnableIRQ(ADC_IRQn);


  // Turn on the ADC
  ADCx->CR2 |= ADC_CR2_ADON;


}



void timer_init(void){

  // Enable Timer 3 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Disable Timer 3
  TIM3->CR1 &= ~TIM_CR1_CEN;

  // Counting Direction: 0 = up-counting, 1 = down-counting
  TIM3->CR1 &=~(TIM_CR1_DIR);

  // Auto-reload preload enable
  TIM3->CR1 &=~(TIM_CR1_ARPE);
  TIM3->CR1 |=(TIM_CR1_ARPE);

  //Clock Prescaler
  uint32_t TIM3COUNTER_Frequency = 28e6; //Desired Frequency
  TIM3->PSC = (84000000/TIM3COUNTER_Frequency)-1;
  //TIM4->PSC = 62499;

  //Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  TIM3->ARR = 999;
  //TIM3->ARR = 1343;

  // ------------------Channel 3 Setup ----------------------------------

  // Disable Input/Output for Channel 3
  // This must be disable in order to set the channel as
  // Input or Output
  TIM3->CCER &= ~TIM_CCER_CC3E;

  // Set Channel 3 as output channel
  TIM3->CCMR2 &= ~(TIM_CCMR2_CC3S);

  // Set the first value to compare against
  // 50% duty cycle
  //TIM4->CCR3=.5*TIM4->ARR;
  TIM3->CCR3 =0 ;

  // Clear Output compare mode bits for channel 3
  // Select Pulse Width Modulation Mode 1
  TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1);

  // Select Preload Enable to be enable for PWM, allow to update CCR3 register
  // to be updated at overflow/underflow events
  TIM3->CCMR2 &=~(TIM_CCMR2_OC3PE);
  TIM3->CCMR2 |= (TIM_CCMR2_OC3PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
  TIM3->CCER &= ~(TIM_CCER_CC3P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
  TIM3->CCER &=~(TIM_CCER_CC3NP);

  // Enable Output for channel 3
  TIM3->CCER |= TIM_CCER_CC3E;


  // ------------------Channel 4 Setup ----------------------------------

  // Disable Input/Output for Channel 4
  // This must be disable in order to set the channel as
  // Input or Output
  TIM3->CCER &= ~TIM_CCER_CC4E;

  // Set Channel 4 as output channel
  TIM3->CCMR2 &= ~(TIM_CCMR2_CC4S);

  // Set the first value to compare against
  // 50% duty cycle
  //TIM4->CCR3=.5*TIM4->ARR;
  TIM3->CCR4 =0 ;

  // Clear Output compare mode bits for channel 3
  // Select Pulse Width Modulation Mode 1
  TIM3->CCMR2 |= (TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1);

  // Select Preload Enable to be enable for PWM, allow to update CCR4 register
  // to be updated at overflow/underflow events
  TIM3->CCMR2 &=~(TIM_CCMR2_OC4PE);
  TIM3->CCMR2 |= (TIM_CCMR2_OC4PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
  TIM3->CCER &= ~(TIM_CCER_CC4P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
  TIM3->CCER &=~(TIM_CCER_CC4NP);

  // Enable Output for channel 4
  TIM3->CCER |= TIM_CCER_CC4E;

  // --------------------------------------------------------------

  // Enable Update Generation
  TIM3->EGR &= ~TIM_EGR_UG;
  TIM3->EGR |= TIM_EGR_UG;

  // Center Align Mode Selection
  TIM3->CR1 &=~(TIM_CR1_CMS);

  //Disable interrupts only on channel 4
  //TIM3->DIER &=~(TIM_DIER_CC4IE);

  // Enable Timer 4 after all of the initialization
  TIM3->CR1 |= TIM_CR1_CEN;

}


int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /* LCD GPIO SETUP */
  /*Enable Clock*/
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOD->MODER &=~( GPIO_MODER_MODE15 | GPIO_MODER_MODE14
                  | GPIO_MODER_MODE13 | GPIO_MODER_MODE12
                  | GPIO_MODER_MODE10 | GPIO_MODER_MODE9
                  | GPIO_MODER_MODE8  | GPIO_MODER_MODE7
                  | GPIO_MODER_MODE6  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3  | GPIO_MODER_MODE2
                  | GPIO_MODER_MODE1  | GPIO_MODER_MODE0);

  GPIOD->MODER |= ( GPIO_MODER_MODE15_0 |  GPIO_MODER_MODE14_0
                  | GPIO_MODER_MODE13_0 |  GPIO_MODER_MODE12_0
                  | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE9_0
                  | GPIO_MODER_MODE8_0  | GPIO_MODER_MODE7_0
                  | GPIO_MODER_MODE6_0  |  GPIO_MODER_MODE4_0
                  | GPIO_MODER_MODE3_0  |  GPIO_MODER_MODE2_0 //output
                  | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE0_0); //output

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~( GPIO_OTYPER_OT15 | GPIO_OTYPER_OT14
                    | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT12
                    | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT9
                    | GPIO_OTYPER_OT8  | GPIO_OTYPER_OT7
                    | GPIO_OTYPER_OT6  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3  | GPIO_OTYPER_OT2
                    | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT0);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9
                    | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED7
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */

  GPIOD->OSPEEDR |= ( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9
                    | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED7
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved

  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD15 | GPIO_PUPDR_PUPD14
                  | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD12
                  | GPIO_PUPDR_PUPD10  | GPIO_PUPDR_PUPD9 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD8  | GPIO_PUPDR_PUPD7 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD6  | GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD3  | GPIO_PUPDR_PUPD2
                  | GPIO_PUPDR_PUPD1  | GPIO_PUPDR_PUPD0);


  /* POTENTIOMETER SETUP */

  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);

 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOC->MODER &=~(GPIO_MODER_MODE1|GPIO_MODER_MODE2);
  GPIOC->MODER |= (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1);
  GPIOC->MODER |= (GPIO_MODER_MODE2_0|GPIO_MODER_MODE2_1);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOC->OTYPER &= ~( GPIO_OTYPER_OT1|GPIO_OTYPER_OT2);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOC->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED2); /* Configure as high speed */
  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED2); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1|GPIO_PUPDR_PUPD2);


  /* PUSH BUTTON SETUP */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOA->MODER &=~(GPIO_MODER_MODE0);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOA->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */
  //GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);
  //GPIOB->PUPDR |= (GPIO_PUPDR_PUPD0_0);



  // Motor driver L205N

  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOA->MODER &=~( GPIO_MODER_MODE7  | GPIO_MODER_MODE6
                  | GPIO_MODER_MODE5  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3  | GPIO_MODER_MODE2);

  GPIOA->MODER |= ( GPIO_MODER_MODE7_0  |  GPIO_MODER_MODE6_0
                  | GPIO_MODER_MODE5_0  |  GPIO_MODER_MODE4_0 //output
                  | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE2_0); //output

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOA->OTYPER &= ~( GPIO_OTYPER_OT7  | GPIO_OTYPER_OT6
                    | GPIO_OTYPER_OT5  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3  | GPIO_OTYPER_OT2);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOA->OSPEEDR &=~(  GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED5
                    | GPIO_OSPEEDR_OSPEED2  | GPIO_OSPEEDR_OSPEED7); /* Configure as high speed */

  GPIOA->OSPEEDR |= (  GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED7  | GPIO_OSPEEDR_OSPEED5); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved

  GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPD6  | GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD3  | GPIO_PUPDR_PUPD2
                  | GPIO_PUPDR_PUPD5  | GPIO_PUPDR_PUPD7);


  GPIOA->BSRR = (0xC);



  /* PWM Alternate Function PB1 SETUP */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);
 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOB->MODER &=~(GPIO_MODER_MODE1|GPIO_MODER_MODE0);
  GPIOB->MODER |=(GPIO_MODER_MODE1_1| GPIO_MODER_MODE0_1);

  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL1|GPIO_AFRL_AFSEL0);
  GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL1_1|GPIO_AFRL_AFSEL0_1);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT1|GPIO_OTYPER_OT0);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOB->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */
  GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD1|GPIO_PUPDR_PUPD0);



  // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);
  timer_init();
  ADCx_Init(ADC1);
  EXTI_Init();
  LCD_init(&rgb_lcd,GPIOD,0,0,1,2,3,4,6,7,8,9,10,4,20,LCD_8BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  LCD_print(&rgb_lcd, "Periphery Setup Complete");
 // RangeMap_init(&motor_foward,4095,2148,0,999);
  //RangeMap_init(&motor_foward,2148,4095,999,0);
  RangeMap_init(&motor_foward,2148,4095,0,999);
  RangeMap_init(&motor_backward,1947,0,0,999);
  GPIOD->ODR ^= PORTD_14;

  LCD_clear(&rgb_lcd);
  //LCD_home(&rgb_lcd);
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);

  while(1){
  ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    /* Enable the selected ADC conversion for regular group */
   while(counter <= 1);
      counter = 0;
      LCD_print(&rgb_lcd, "ADC POT1: %4d", pot_adc1);
      LCD_setCursor(&rgb_lcd, 0,1);
      LCD_print(&rgb_lcd, "ADC POT2: %4d", pot_adc2);
      LCD_setCursor(&rgb_lcd, 0,2);
      uint16_t output=motor_pwm(&motor_foward,&motor_backward,pot_adc2);
      uint16_t output1=motor_pwm_rl(pot_adc1);
      LCD_print(&rgb_lcd, "MOTOR PWM2: %4d", output);
      LCD_setCursor(&rgb_lcd, 0,3);
      LCD_print(&rgb_lcd, "MOTOR PWM1: %4d", output1);
      LCD_home(&rgb_lcd);
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
