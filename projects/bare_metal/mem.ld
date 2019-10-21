/* Memory layout for the STM32F407 */

/* Entry Point
* Doesn't really matter but informative for debuggers,etc
*/
ENTRY(_start)

MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}


/* Output Sectons */
SECTIONS {
  /* Text section (code and read-only data)*/
  .text : {
     . = ALIGN(4);
    /*Insert ISR vector table first*/
    *(.isr_vector)
    /* test section (code) */
    *(.text)
    *(.text*)
     . = ALIGN(4);
    KEEP(*(.isr_vector))
  } > FLASH


  .rodata :  {
     . = ALIGN(4);

    /* .rodata sections (constants) */
    *(.rodata)

    /* .rodata* sections (strings, etc.) */
    *(.rodata*)
     . = ALIGN(4);
  } > FLASH

  /* Used by the startup to initialize data */
  /* this contains the LMA of the data section */
  _sidata=LOADADDR(.data);

  .data : ALIGN(4) {
    . = ALIGN(4);
    _sdata = .; /* create a global symbol at data start */

    *(.data)
    *(.data*)

    . = ALIGN(4);
    _edata = .;
  } > RAM AT>FLASH



  .bss : ALIGN(4) {
    /* This is used by the startup in order to initialize the .bss section */
     . = ALIGN(4);
    _sbss = .; /* define a global symbol at bss start */
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    /* define a global symbol at bss end */
    _ebss = .;
  } >RAM
}
