#ifndef __SCHEDULER_
#define __SCHEDULER_

#include "stm32f407xx.h"
#define wub() SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

/* Calls the PENDSV ISR */
void schedule(void);

#endif /*__SCHEDULER_*/
