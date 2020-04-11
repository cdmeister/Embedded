#ifndef LED_H
#define LED_H

#include "stm32f407xx.h"

void blue_led_off(void);
void blue_led_on(void);
void blue_led_toggle(void);

void red_led_off(void);
void red_led_on(void);
void red_led_toggle(void);

void ledInit(void);



#endif
