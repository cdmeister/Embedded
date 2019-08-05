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
#include "CS43L22.h"
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
volatile uint32_t sampleCounter = 0;

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
  while (1)
  {
    if((SPI3->SR & SPI_SR_TXE)){
			SPI3->DR = sampleCounter;
    	//only update on every second sample to insure that L & R ch. have the same sample value
    		sampleCounter++;
		}
		if (sampleCounter == 65535)
   	{
   		sampleCounter = 0;

   	}
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
