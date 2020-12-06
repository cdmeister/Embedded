#ifndef __SCHEDULER_
#define __SCHEDULER_

#include "stm32f407xx.h"
/* Calls the PENDSV ISR */
#define schedule() SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;


#endif /*__SCHEDULER_*/
