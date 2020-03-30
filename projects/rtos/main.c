#include "stm32f407xx.h"
#include "systick.h"
#include "task_management.h"
#include "scheduler.h"

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */
extern struct task_block TASKS[MAX_TASKS];

#define kernel TASKS[0]


void blue_led_off(void){

  GPIOD->BSRR |= GPIO_BSRR_BR15;
}
void blue_led_on(void){

  GPIOD->BSRR |= GPIO_BSRR_BS15;
}

void blue_led_toggle(void){
  GPIOD->ODR ^= GPIO_ODR_ODR_15;
}

void red_led_off(void){

  GPIOD->BSRR |= GPIO_BSRR_BR14;
}
void red_led_on(void){

  GPIOD->BSRR |= GPIO_BSRR_BS14;
}

void red_led_toggle(void){
  GPIOD->ODR ^= GPIO_ODR_ODR_14;
}



void task_test0(void *arg)
{
    uint32_t now = millis();
    /*blue_led_on();*/
    while(1) {
        if ((millis() - now) > 1000) {

            /*blue_led_off();
            schedule();*/
            now = millis();
            /*blue_led_on();
            */
            blue_led_toggle();
        }
    }
}

void task_test1(void *arg)
{
    uint32_t now = millis();
    red_led_on();
    while(1) {
        if ((millis() - now) > 1000) {
            /*red_led_off();
            schedule();
            now = millis;
            red_led_on();
            */
            red_led_toggle();
        }
    }
}


int main() {
  /*uint32_t * stack_end = &_os_stack_stop;*/
  SysTick_Init(SystemCoreClock/1000);
    /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;


  /* Set mode of all pins as digital output
   *  00 = digital input       01 = digital output
   *  10 = alternate function  11 = analog (default)
   */
  GPIOD->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13
                   | GPIO_MODER_MODE14 | GPIO_MODER_MODE15); /* Clear mode bits */
  GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0
                   | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);/* LED 5-8 are on GPIOD Pins 12-15 */

  /* Set output type of all pins as push-pull
   * 0 = push-pull (default)
   * 1 = open-drain
   */
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 |
                   GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15); /*Configure as output open-drain */

  /* Set output speed of all pins as high
   * 00 = Low speed           01 = Medium speed
   * 10 = Fast speed          11 = High speed
   */
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | /* Configure as high speed */
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);

  /* Set all pins as no pull-up, no pull-down
   * 00 = no pull-up, no pull-down    01 = pull-up
   * 10 = pull-down,                  11 = reserved
   */
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13
                  | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15); /*no pul-up, no pull-down*/


  kernel.name[0]= 0;
  kernel.id = 0;
  kernel.state = TASK_RUNNING;
  task_create("test0",task_test0,NULL);
  /*task_create("test1",task_test1,NULL);
  while(1){
    schedule();
  }
  */

  /* Infinite loop */
  while (1)
  {
  }


  return 0;
}

