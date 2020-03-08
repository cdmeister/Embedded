#include "scheduler.h"

extern int running_task_id;
void schedule(){
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}



void PendSV_Handler(void){

    store_context();
    __asm__ volatile("mrs %0, msp" : "=r"(TASKS[running_task_id].sp));
    TASKS[running_task_id].state = TASK_WAITING;
    running_task_id++;
    if (running_task_id >= n_tasks)
        running_task_id = 0;
    TASKS[running_task_id].state = TASK_RUNNING;
    __asm__ volatile("msr msp, %0" ::"r"(TASKS[running_task_id].sp));
    restore_context();
    __asm__ volatile("mov lr, %0" ::"r"(0xFFFFFFF9));
    __asm__ volatile("bx lr");
}
