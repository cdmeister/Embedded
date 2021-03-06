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
#include "systick.h"
#include "lcd.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC_TEMPERATURE_V25       760  /* mV */
#define ADC_TEMPERATURE_AVG_SLOPE 2500 /* mV/C */
#define NUM_SAMPLES               10

const double BETA               = 3435.0;
const double ROOM_TEMP          = 298.15;   // room temperature in Kelvin
const double RESISTOR_ROOM_TEMP = 10000.0;

uint8_t counter =0;
uint16_t sample_array[10];
double temperature = 0;
uint16_t temperature_adc = 0;
uint8_t ready = 0;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
double adc_value_to_temp(const uint16_t value) {

  double rThermistor = ((4096/(double) value) - 1);
  rThermistor = 10000/rThermistor;

  double steinhart;
  steinhart = rThermistor / RESISTOR_ROOM_TEMP;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BETA;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (ROOM_TEMP); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
}


void ADC_IRQHandler(void){
  if((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC){
    /* acknowledge interrupt */
    sample_array[counter] = ADC1->DR;
    counter++;
    if(counter == 9){
      int sum=0, i;
      for( i=0;i<NUM_SAMPLES;i++){
        sum += sample_array[i];
      }
      temperature_adc = sum / NUM_SAMPLES;
      counter =0;
      ready = 1;
    }

  }

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

  // Disable Continuos Mode
  ADCx->CR2 &= ~(ADC_CR2_CONT);
  ADCx->CR2 |= (ADC_CR2_CONT);

  // External Trigger on rising edge
  ADCx->CR2 &= ~(ADC_CR2_EXTEN);

  // Timer 2 Trigger to drive ADC conversion
  ADCx->CR2 &= ~(ADC_CR2_EXTSEL);

  // Data Alignment
  ADCx->CR2 &= ~(ADC_CR2_ALIGN);

  // Number of Conversions
  ADCx->SQR1 &= ~(ADC_SQR1_L);
  //ADCx->SQR1 |= (ADC_SQR1_L_0); // 2 conversion


  // Enable Temperature/Vref
  ADC123_COMMON->CCR &=~(ADC_CCR_TSVREFE);


  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  // PC1 is connected to ADC channel 11
  ADCx->SQR3 |= (ADC_SQR3_SQ1_3|ADC_SQR3_SQ1_1|ADC_SQR3_SQ1_0);
  // Sample Time is 480 cycles
  ADCx->SMPR1 &= ~(ADC_SMPR1_SMP11);
  ADCx->SMPR1 |= ADC_SMPR1_SMP11;

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

  ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;

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

  // Enable GPIO clock and configure the Tx pin and the Rx pin as
  // Alternating functions, High Speed, Push-pull, Pull-up

  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOCEN);

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

  GPIOC->MODER &=~(GPIO_MODER_MODE1);
  GPIOC->MODER |= (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1);

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


  GPIOC->OTYPER &= ~( GPIO_OTYPER_OT1);

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


  GPIOC->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */

  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */


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


  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1);

    // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);

  ADCx_Init(ADC1);

  LCD rgb_lcd;
  LCD_init(&rgb_lcd,GPIOD,0,0,1,2,3,4,6,7,8,9,10,4,20,LCD_8BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  LCD_print(&rgb_lcd, "Periphery Setup Complete");
  Delay(1000);

  LCD_clear(&rgb_lcd);
  //LCD_home(&rgb_lcd);
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);

  while(1){
    /* Enable the selected ADC conversion for regular group */
    while(!ready); // Wait till conversion is done
    ready = 0;
    //float temp = adc_value_to_temp(temperature_adc);
    LCD_print(&rgb_lcd, "ADC TEMP: %4d", temperature_adc);
    float temp_thermistor = adc_value_to_temp(temperature_adc);
    LCD_setCursor(&rgb_lcd, 0,1);
    LCD_print(&rgb_lcd,"TEMP: %4.2f\xDF%1c", temp_thermistor,'C');

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

