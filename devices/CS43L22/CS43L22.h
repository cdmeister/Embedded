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
#include "systick.h"
/* Exported types ------------------------------------------------------------*/
uint16_t CS43L22_address = 0x94;
/* Exported constants --------------------------------------------------------*/

/* Delay for the Codec to be correctly reset */
#define CS43L22_RESET_DELAY               0x4FFF

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void CS43L22_Init();
void CS43L22_writeReg(uint8_t address, uint8_t data);
uint8_t CS43L22_readReg(uint8_t address);
void CS43L22_powerDown();
void CS43L22_powerUp();
void CS43L22_reset();
void CS43L22_activate();
static void CS43L22_GPIO_init();
static void CS43L22_I2C_init();
static void CS43L22_I2S_init();



#endif /* __CS43L22_H */
