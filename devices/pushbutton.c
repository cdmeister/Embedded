#include "pushbutton.h"

static volatile uint16_t pot_adcx = 0;
static volatile uint16_t pot_adcy = 0;

static volatile uint8_t counter =0;

void ADC_IRQHandler(void){
  if((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC){
    /* acknowledge interrupt */
    uint16_t value;
    value = ADC1->DR;
    if(counter == 0) {
      pot_adcx = value;
    }
    else {
      pot_adcy = value;
    }
    counter^=counter;
  }

}

uint16_t getADCx_coordinate(void){
  return pot_adcx;
}

uint16_t getADCy_coordinate(void){
  return pot_adcy;
}

uint32_t pushbutton_adc_init(ADC_TypeDef * ADCx){

  rcc_adc_enable(ADCx);

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
  ADC123_COMMON->CCR &= ~(ADC_CCR_ADCPRE);  /* Clear*/
  ADC123_COMMON->CCR |= (ADC_CCR_ADCPRE_0); /* DIV4*/


  /* Disable DMA */
  ADC123_COMMON->CCR &= ~(ADC_CCR_DMA);

  /*Configurable delay between conversions in Dual/Triple interleaved mode*/
  ADC123_COMMON->CCR &= ~(ADC_CCR_DELAY);

  /* Resolution ot 12-bits*/
  ADCx->CR1 &= ~(ADC_CR1_RES);

  /* Scan Mode for this example*/
  ADCx->CR1 &=~( ADC_CR1_SCAN);
  ADCx->CR1 |=( ADC_CR1_SCAN);

  /* Enable Continuos Mode*/
  ADCx->CR2 |= (ADC_CR2_CONT);

  /* External Trigger on rising edge */
  ADCx->CR2 &= ~(ADC_CR2_EXTEN);

  /* No Timer Trigger to drive ADC conversion */
  ADCx->CR2 &= ~(ADC_CR2_EXTSEL);

  /* Data Alignment*/
  ADCx->CR2 &= ~(ADC_CR2_ALIGN);

  /* Number of Conversions*/
  ADCx->SQR1 &= ~(ADC_SQR1_L);
  /* 2 conversion */
  ADCx->SQR1 |= (ADC_SQR1_L_0);

  /* Disable Temperature/Vref*/
  ADC123_COMMON->CCR &=~(ADC_CCR_TSVREFE);


  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  /* PC1 is connected to ADC channel 11 */
  ADCx->SQR3 |= (ADC_SQR3_SQ1_3|ADC_SQR3_SQ1_1|ADC_SQR3_SQ1_0);
  /* Sample Time is 480 cycles*/
  ADCx->SMPR1 |= (ADC_SMPR1_SMP11);

  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ2);
  /* PC2 is connected to ADC channel 12*/
  ADCx->SQR3 |= (ADC_SQR3_SQ2_3|ADC_SQR3_SQ2_2);
  /* Sample Time is 480 cycles*/
  ADCx->SMPR1 |= (ADC_SMPR1_SMP12);


  /* This call enables the end-of-conversion flag after each channel,
   * which triggers the end-of-conversion interrupt every time this flag is set.
   */
  ADCx->CR2 |= ADC_CR2_EOCS;

  /* Enable Regular channel Interrupt*/
  ADCx->CR1 |= ADC_CR1_EOCIE;


  /* Set ADCx priority to 1*/
  NVIC_SetPriority(ADC_IRQn,1);

  /* Enable ADCx interrupt*/
  NVIC_EnableIRQ(ADC_IRQn);


  /* Turn on the ADC */
  ADCx->CR2 |= ADC_CR2_ADON;
    /* Enable the selected ADC conversion for regular group */
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
  return 0;
}

uint32_t pushbutton_gpio_init(GPIO_TypeDef * GPIOx){

  /* POTENTIOMETER SETUP */
  rcc_gpio_enable(GPIOx);
  /*   Set mode of all pins as digital output
   *   00 = digital input         01 = digital output
   *   10 = alternate function    11 = analog (default)
   */
  GPIOx->MODER &=~(GPIO_MODER_MODE1|GPIO_MODER_MODE2);
  GPIOx->MODER |= (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1);
  GPIOx->MODER |= (GPIO_MODER_MODE2_0|GPIO_MODER_MODE2_1);



  /*  Set output tupe of all pins as push-pull
   *  0 = push-pull (default)
   *  1 = open-drain
   */

  GPIOx->OTYPER &= ~( GPIO_OTYPER_OT1|GPIO_OTYPER_OT2);


  /*  Set output speed of all pins as high
   *  00 = Low speed           01 = Medium speed
   *  10 = Fast speed          11 = High speed
   */

  GPIOx->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED2); /* Configure as high speed */
  GPIOx->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED2); /* Configure as high speed */


  /* Set all pins as no pull-up, no pull-down
   * 00 = no pull-up, no pull-down    01 = pull-up
   * 10 = pull-down,                  11 = reserved
   */
  GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD1|GPIO_PUPDR_PUPD2);
  return 0;
}

uint32_t pushbutton_init(GPIO_TypeDef * GPIOx, ADC_TypeDef * ADCx){

  pushbutton_gpio_init(GPIOx);
  pushbutton_adc_init(ADCx);
  return 0;
}
