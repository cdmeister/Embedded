/**
  ******************************************************************************
  * @file    helper.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    2-July-2019
  * @brief   i2c module
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
inline void I2C_clock_init(I2C_TypeDef * I2Cx);
inline void I2C_CR1_mode(I2C_TypeDef * I2Cx);
inline void I2C_OAR_init(I2C_TypeDef * I2Cx);
#endif /* __I2C_H */
