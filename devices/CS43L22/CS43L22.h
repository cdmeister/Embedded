/**
  ******************************************************************************
  * @file    CS43L22
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    14-JULY-2019
  * @brief   Header for CS43L22
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CS43L22_H
#define __CS43L22_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void CS43L22_init();
void CS43L22_writeReg();
void CS43L22_readReg();
static void CS43L22_GPIO_init();
static void CS43L22_I2C_init();
static void CS43L22_I2S_init();

#endif /* __CS43L22_H */
