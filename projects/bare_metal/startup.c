extern void * _estack;

int main(void);
void Reset_Handler(void)
{

  main();
}

/* This is the main program loop. */
int main(void){

  int i = 0;
  /* Infinite loop */
  while (1)
  {
    i++;
  }

  return 0;
}


/* This is the interrupt vector.
 *
 * It sits at the beginning of the flash image,
 * and contains the initial stack pointer, and
 * the service routines associated to exceptions
 * and interrupt requests.
 *
 * Replace the default empty service routine with
 * the corresponding handler.
 */
__attribute__ ((section(".isr_vector")))
void (* const IV[])(void) =
{
    (void (*)(void))(&_estack),
    Reset_Handler                   // Reset
};
