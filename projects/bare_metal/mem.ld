/* Memory layout for the STM32F407 */

/* Entry Point
* Doesn't really matter but informative for debuggers,etc
*/
ENTRY(Reset_Handler)

MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Output Sectons */
SECTIONS {
  /* Text section (code and read-only data)*/
  .text : {
    . = ALIGN(4); /* section is word aligned */
    /*Insert ISR vector table first*/
    KEEP(*(.isr_vector))
    /* test section (code) */
    *(.text)
  } > FLASH

  .data : {
    *(.data)
  } > RAM
}
