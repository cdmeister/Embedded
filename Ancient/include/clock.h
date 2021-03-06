/**
  ******************************************************************************
  * @file    lcd.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    30-April-2018
  * @brief   Header for clocks.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLOCK_H
#define __CLOCK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"
#include "flash.h"
#include "systick.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern void SystemHSIenable(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler);
extern void SystemHSEenable(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler);

extern void SystemPLLClockEnable(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler,
                            uint8_t pll_m, uint16_t pll_n,
                            uint8_t pll_p, uint8_t pll_q);

extern void SystemPLLClockUpdate(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler,
                            uint8_t pll_m, uint16_t pll_n,
                            uint8_t pll_p, uint8_t pll_q);

extern void SystemClockPrescaler(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler);

extern void SystemPLLPrescaler(uint8_t pll_m, uint16_t pll_n,
                              uint8_t pll_p, uint8_t pll_q);

extern uint16_t GetAHBPrescaler(void);
extern uint8_t GetAPB1Prescaler(void);
extern uint8_t GetAPB2Prescaler(void);
extern uint32_t GetHCLK(void);
extern uint32_t GetPCLK1(void);
extern uint32_t GetPCLK2(void);
extern uint32_t GetSystemClock(void);
extern uint16_t GetPLLI2SR(void);
extern uint16_t GetPLLI2SN(void);
extern uint16_t GetPLLM(void);
extern void SetPLLI2SPrescaler(uint16_t plli2s_n, uint16_t plli2s_r);
extern void SystemPLLI2SEnable(uint16_t plli2s_n, uint16_t plli2s_r);

#endif /* __CLOCK_H */
