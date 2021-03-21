/**
  ******************************************************************************
  * @file    main.c
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    17-September-2017
  * @brief   Main program body
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "startup.h"
#include "systick.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORTD_12 0x00001000 /*Green */
#define PORTD_13 0x00002000 /*Orange */
#define PORTD_14 0x00004000 /*Red */
#define PORTD_15 0x00008000 /*Blue */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void cam_gpio_clock_init(){
  /* Enable Peripheral Clock for GPIOA */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);

 /* Set mode of all pins as digital output */
 /* 00 = digital input         01 = digital output */
 /* 10 = alternate function    11 = analog (default) */
  GPIOA->MODER &=~(GPIO_MODER_MODE8);
  GPIOA->MODER |=(GPIO_MODER_MODE8_1);

  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8);

  /* Set output tupe of all pins as push-pull */
  /* 0 = push-pull (default) */
  /* 1 = open-drain */
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT8);

  /* Set output speed of all pins as high */
  /* 00 = Low speed           01 = Medium speed */
  /* 10 = Fast speed          11 = High speed */
  GPIOA->OSPEEDR |=(GPIO_OSPEEDR_OSPEED8);


  /* Set all pins as no pull-up, no pull-down */
  /* 00 = no pull-up, no pull-down    01 = pull-up */
  /* 10 = pull-down,                  11 = reserved */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD8);

  RCC->CFGR &= ~(RCC_CFGR_MCO1);
  RCC->CFGR |= (RCC_CFGR_MCO1_1);
}

/* Reset Pin GPIO */
void cam_gpio_reset_init(){

 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOD->MODER &=~(GPIO_MODER_MODE8);
  GPIOD->MODER |= (GPIO_MODER_MODE8_0);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT8);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED8);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD8);

}

// RESET Pin to LOW
void cam_reset(){

  GPIOD->BSRR = GPIO_BSRR_BR8;

}

// RESET Pin to HIGH
void cam_activate(){

  GPIOD->BSRR = GPIO_BSRR_BS8;

}


/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

void I2C_init(){

  /* Enable I2C Periphal bus */
  RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN);

  /* Disable I2C for configuration(Espicialy for TRISE) */
  I2C1->CR1 &= ~(I2C_CR1_PE);

  /* Peripheral clock frequncy */
  /* This value must match the AHB frequency sincd I2C is conntect to AHB1 Bus */
  I2C1->CR2 &= ~(I2C_CR2_FREQ);
  I2C1->CR2 |= (I2C_CR2_FREQ_5|I2C_CR2_FREQ_3|I2C_CR2_FREQ_1);

  /* Pick Standard or Fast mode (Master Mode Selection) */
  I2C1->CCR &= ~(I2C_CCR_FS);

  /* IC2 Duty Cycle (Only for Fast Mode) */
  I2C1->CCR &= ~(I2C_CCR_DUTY);

  /* Standard mode speed calculate */
  uint32_t pclk1 = 42000000;
  uint16_t result = (uint16_t)(pclk1 / (100000 << 1));

  /* Test if CCR value is under 0x4 */
  if (result < 0x04)
  {
    /* Set minimum allowed value */
    result = 0x04;
  }
  /* Set speed value for standard mode */
  I2C1->CCR &= ~(I2C_CCR_CCR);
  I2C1->CCR |= result;

  /* Set Maximum Rise Time for standard mode */
  I2C1->TRISE = (pclk1/1000000) + 1;
  /* Select Mode I2C mode */
  I2C1->CR1 &= ~(I2C_CR1_SMBUS);

  // Enable I2C to generate ACK
  I2C1->CR1 &= ~(I2C_CR1_ACK);
  I2C1->CR1 |= (I2C_CR1_ACK);

  /* Enable the selected I2C peripheral */
  I2C1->CR1 |= I2C_CR1_PE;

}

void I2C_GPIO_init(void){

  /* ---- Setup the GPIO Pins directly connect to I2C */
  /* ---- I2C: PB6 and PB9 are already pulled up so you don't need any */
  /* ---- internal pull up resistor. Configure it as output and open drain */

  /* Enable Peripheral Clock for GPIOB */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);

 /* Set mode of all pins as digital output */
 /* 00 = digital input         01 = digital output */
 /* 10 = alternate function    11 = analog (default) */
  GPIOB->MODER &=~(GPIO_MODER_MODE6|GPIO_MODER_MODE9);
  GPIOB->MODER |=(GPIO_MODER_MODE6_1| GPIO_MODER_MODE9_1);

  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6);
  GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL6_2);

  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
  GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL9_2);


  /* Set output tupe of all pins as push-pull */
  /* 0 = push-pull (default) */
  /* 1 = open-drain */
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6|GPIO_OTYPER_OT9);
  GPIOB->OTYPER |=  (GPIO_OTYPER_OT6|GPIO_OTYPER_OT9);

  /* Set output speed of all pins as high */
  /* 00 = Low speed           01 = Medium speed */
  /* 10 = Fast speed          11 = High speed */
  GPIOB->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED9);


  /* Set all pins as no pull-up, no pull-down */
  /* 00 = no pull-up, no pull-down    01 = pull-up */
  /* 10 = pull-down,                  11 = reserved */
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6|GPIO_PUPDR_PUPD9);

}
void I2C_writeReg(uint16_t address, uint8_t slave_address, uint16_t data, uint8_t size ){

  // Send the START generation
  I2C1->CR1 |= I2C_CR1_START;

  while(!(I2C1->SR1 & I2C_SR1_SB)){
    // Wait for start condition to be generated
  }

  I2C1->DR = (slave_address); // Send the chip address
  while (!(I2C1 ->SR1 & I2C_SR1_ADDR )) {}// Wait for master transmitter mode.
	I2C1 ->SR2; // Must read SR2 to clear ADDR bit

  // TxE=1 , shift register empty, data register empty, write Data1 in DR
  I2C1->DR = (address>>8); // Transmit the address to write to.
  while(!(I2C1->SR1 & I2C_SR1_TXE)){}

  I2C1->DR = (address&0xFF); // Transmit the address to write to.

  while(!(I2C1->SR1 & I2C_SR1_TXE)){}
  if( size == 8){
    I2C1->DR = (data); // Transmit the address to write to.
  }
  else {
    I2C1->DR = (data >> 8); // Transmit the address to write to.
    while(!(I2C1->SR1 & I2C_SR1_TXE)){}
    I2C1->DR = (data & 0xFF); // Transmit the address to write to.
  }
  // After the last tranmission of data
  while ((I2C1 ->SR1 & I2C_SR1_BTF ) == 0){} // Wait for all bytes to finish.
	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.

  return;
}
uint8_t I2C_readReg(uint16_t address, uint8_t slave_address ){

  // Send the START generation
  I2C1->CR1 |= I2C_CR1_START;

  while(!(I2C1->SR1 & I2C_SR1_SB)){
    // Wait for start condition to be generated
  }

  I2C1->DR = (slave_address); // Send the chip address
  while (!(I2C1 ->SR1 & I2C_SR1_ADDR )) {}// Wait for master transmitter mode.
	I2C1 ->SR2; // Must read SR2 to clear ADDR bit

  // TxE=1 , shift register empty, data register empty, write Data1 in DR
  I2C1->DR = (address>>8); // Transmit the address to write to.
  while(!(I2C1->SR1 & I2C_SR1_TXE)){}

  I2C1->DR = (address&0xFF); // Transmit the address to write to.

  // After the last tranmission of data
  while ((I2C1 ->SR1 & I2C_SR1_BTF ) == 0){} // Wait for all bytes to finish.
	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.

  // Read Sequence
  I2C1->CR1 |= I2C_CR1_START;
   while(!(I2C1->SR1 & I2C_SR1_SB)){
    // Wait for start condition to be generated
  }

  I2C1->DR = (slave_address|0x1); // Send the chip address
  while (!(I2C1 ->SR1 & I2C_SR1_ADDR )) {}// Wait for master reciver mode.

  // Disable I2C to generate ACK
  I2C1->CR1 &= ~(I2C_CR1_ACK);
	I2C1->SR2; // Must read SR2 to clear ADDR bit

  // Wait for byte to move into data register.
	while(!(I2C1->SR1 & I2C_SR1_RXNE)){}

  uint8_t recieved_data = I2C1->DR;


	I2C1->CR1 |= I2C_CR1_STOP; // End the transfer sequence.

  // Re-Enable I2C to generate ACK for next transaction
  I2C1->CR1 |= (I2C_CR1_ACK);

  // Since NACK is sent after the last data the AF flag in SR1 is active
  // we are going to clear it
  I2C1->SR1 &=~(I2C_SR1_AF);

  return recieved_data;
}


int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;


  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOD->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13
                   | GPIO_MODER_MODE14 | GPIO_MODER_MODE15); /* Clear mode bits */
  GPIOD->MODER |= (GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0
                   | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0);/* LED 5-8 are on GPIOD Pins 12-15 */

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 |
                   GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15); /*Configure as output open-drain */

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | /* Configure as high speed */
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13
                  | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15); /*no pul-up, no pull-down*/

  /* Generate and interrupt every 1ms */
  /* http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html */
  /* If the clock is at 168MHz, then that is 168 000 000 ticks per second */
  /* but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead */
  /* you can generate an interupt every 1ms so that would be 168 000 ticks per */
  /* ms and you can fit 168 000 ticks into the LOAD register */
  SysTick_Init(SystemCoreClock/1000);

  I2C_GPIO_init();
  I2C_init();
  cam_gpio_reset_init();
  cam_activate();
  Delay(10);
  cam_gpio_clock_init();

  uint8_t chipIDHigh = I2C_readReg(0x300A,0x78);
  uint8_t pidIDLow = I2C_readReg(0x300B,0x78);
  uint8_t slew  = I2C_readReg(0x3604,0x78);
  uint8_t gain  = I2C_readReg(0x4400, 0x78);
  uint8_t gain2  = I2C_readReg(0x4401, 0x78);
  I2C_writeReg(0x4400, 0x78,0x3E, 8 );
  gain  = I2C_readReg(0x4400, 0x78);
  gain2  = I2C_readReg(0x4401, 0x78);
  if(chipIDHigh == 0x56 && pidIDLow == 0x40){
    GPIOD->ODR ^= PORTD_15;
  }

  while(1){

  }
  return 0;
}
