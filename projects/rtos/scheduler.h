#ifndef __SCHEDULER_
#define __SCHEDULER_

#include "stm32f407xx.h"

/* Calls the PENDSV ISR */
void schedule(void);

#endif /*__SCHEDULER_*/
