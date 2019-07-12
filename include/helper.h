/**
  ******************************************************************************
  * @file    helper.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    2-July-2019
  * @brief   Random Functions
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HELPER_H
#define __HELPER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "math.h"
/* Exported types ------------------------------------------------------------*/
typedef struct RangeMap{

  double _slope;
  double _input_start,_input_end;
  double _output_start,_output_end;

}RangeMap;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RangeMap_init(RangeMap * RangeMapx,double input_start, double input_end,
                    double output_start,double output_end);

double RangeMap_map(RangeMap* RangeMapx, double input);
#endif /* __HELPER_H */
