#include "stm32f407xx.h"
#include "systick.h"
#include "task_management.h"
#include "scheduler.h"
#include "led.h"

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */
extern struct task_block TASKS[MAX_TASKS];

#define kernel TASKS[0]



void task_test0(void *arg)
{
    uint32_t now = millis();
    blue_led_on();
    while(1) {
        if ((millis() - now) > 1000) {

            blue_led_off();
            /*schedule();*/
            wub();
            now = millis();
            blue_led_on();

            /*blue_led_toggle();*/
        }
    }
}

void task_test1(void *arg)
{
    uint32_t now = millis();
    red_led_on();
    while(1) {
        if ((millis() - now) > 1000) {
            red_led_off();
            /*schedule();*/
            wub();
            now = millis();
            red_led_on();

            /*red_led_toggle();*/
        }
    }
}


int main() {
  /*uint32_t * stack_end = &_os_stack_stop;*/
  SysTick_Init(SystemCoreClock/1000);
  ledInit();
  kernel.name[0]= 0;
  kernel.id = 0;
  kernel.state = TASK_RUNNING;
  task_create("test0",task_test0,NULL);
  task_create("test1",task_test1,NULL);
  while(1){
    wub();
    /*schedule();*/
  }

  return 0;
}

