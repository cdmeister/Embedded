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

const uint16_t AUDIO_SAMPLE[] = {
0x4952, 0x4646, 0x4f6e, 0xf, 0x4157, 0x4556, 0x6d66, 0x2074, 0x12, 0, 0x1,
0x2, 0xbb80, 0, 0xee00, 0x2, 0x4, 0x10, 0, 0x6166, 0x7463, 0x4,
0, 0xd3cf, 0x3, 0x6164, 0x6174, 0x4f3c, 0xf,-1};


int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
  SysTick_Init(SystemCoreClock/1000);
  CS43L22_Init();

   /* Infinite loop */
  int i =0;
  while (1)
  {
    while(!(SPI3->SR & SPI_SR_TXE));
    if(AUDIO_SAMPLE[i] >-1){
      SPI3->DR = AUDIO_SAMPLE[i];
      i++;

    }
    else{i=0;}
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
