#include "CS43L22.h"

static void CS43L22_GPIO_init();
static void CS43L22_I2C_init();
static void CS43L22_I2S_init();

uint16_t CS43L22_address = 0x94;

void CS43L22_Init(){

  // Setup All of the GPIO Pins for
  //  1. I2C1
  //  2. I2S3
  //  3. CS43L22 Audio Reset
  CS43L22_GPIO_init();


  // Setup the I2C1 module
  CS43L22_I2C_init();
  CS43L22_I2S_init();
  CS43L22_powerUp();
  return;
}

static void CS43L22_GPIO_init(){

  // ---- Setup the GPIO Pins directly connect to CS43L22
  // ---- I2C: PB6 and PB9 are already pulled up so you don't need any
  // ---- internal pull up resistor. Configure it as output and open drain

  /* Enable Peripheral Clock for GPIOB */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN);

 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOB->MODER &=~(GPIO_MODER_MODE6|GPIO_MODER_MODE9);
  GPIOB->MODER |=(GPIO_MODER_MODE6_1| GPIO_MODER_MODE9_1);

  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6);
  GPIOB->AFR[0] |= (GPIO_AFRL_AFSEL6_2);

  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
  GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL9_2);


  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6|GPIO_OTYPER_OT9);
  GPIOB->OTYPER |=  (GPIO_OTYPER_OT6|GPIO_OTYPER_OT9);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOB->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED9);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6|GPIO_PUPDR_PUPD9);

  // ---- Setup the GPIO Pins directly connect to CS43L22
  // ---- PD4: Reset Pin

  /* Enable Peripheral Clock for GPIOD */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN);

 // Set mode of all pins as digital output
 // 00 = digital input         01 = digital output
 // 10 = alternate function    11 = analog (default)
  GPIOD->MODER &=~(GPIO_MODER_MODE4);
  GPIOD->MODER |= (GPIO_MODER_MODE4_0);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT4);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED4);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD4);


  // ---- I2S GPIO Pin Setup

  /* Enable Peripheral Clock for GPIOC */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN);

  // PC7: I2S3_MCK
  // PC10: I2S3_SCK
  // PC12: I2S3_SD
  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOC->MODER &=~(GPIO_MODER_MODE7|GPIO_MODER_MODE10|GPIO_MODER_MODE12);
  GPIOC->MODER |=(GPIO_MODER_MODE7_1| GPIO_MODER_MODE10_1|GPIO_MODER_MODE12_1);

  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL7);
  GPIOC->AFR[0] |= (GPIO_AFRL_AFSEL7_1|GPIO_AFRL_AFSEL7_2);

  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10|GPIO_AFRH_AFSEL12);
  GPIOC->AFR[1] |= (GPIO_AFRH_AFSEL10_1|GPIO_AFRH_AFSEL10_2|
                    GPIO_AFRH_AFSEL12_1|GPIO_AFRH_AFSEL12_2);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT7|GPIO_OTYPER_OT10|GPIO_OTYPER_OT12);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOC->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED7|GPIO_OSPEEDR_OSPEED10
                    |GPIO_OSPEEDR_OSPEED12);


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD7|GPIO_PUPDR_PUPD10|GPIO_PUPDR_PUPD12);

  // PA4: I2S3_WS
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOA->MODER &=~( GPIO_MODER_MODE4);
  GPIOA->MODER |=( GPIO_MODER_MODE4_1); //Alternate Function

  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL4_1|GPIO_AFRL_AFSEL4_2);


  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOA->OTYPER &= ~( GPIO_OTYPER_OT4);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOA->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED4); /* Configure as low speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPD4);

  return;
}

static void CS43L22_I2C_init(){

  // Enable I2C Periphal bus
  RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN);

  // Disable I2C for configuration(Espicialy for TRISE)
  I2C1->CR1 &= ~(I2C_CR1_PE);

  // Peripheral clock frequncy
  // This value must match the AHB frequency sincd I2C is conntect to AHB1 Bus
  I2C1->CR2 &= ~(I2C_CR2_FREQ);
  I2C1->CR2 |= (I2C_CR2_FREQ_5|I2C_CR2_FREQ_3|I2C_CR2_FREQ_1);

  // Pick Standard or Fast mode (Master Mode Selection)
  I2C1->CCR &= ~(I2C_CCR_FS);

  // IC2 Duty Cycle (Only for Fast Mode)
  I2C1->CCR &= ~(I2C_CCR_DUTY);

  // Standard mode speed calculate
  uint32_t pclk1 = GetPCLK1();
  uint16_t result = (uint16_t)(pclk1 / (100000 << 1));

  // Test if CCR value is under 0x4
  if (result < 0x04)
  {
    // Set minimum allowed value
    result = 0x04;
  }
  // Set speed value for standard mode
  I2C1->CCR &= ~(I2C_CCR_CCR);
  I2C1->CCR |= result;

  // Set Maximum Rise Time for standard mode
  I2C1->TRISE = (pclk1/1000000) + 1;
  // Select Mode I2C mode
  I2C1->CR1 &= ~(I2C_CR1_SMBUS);

  // Enable I2C to generate ACK
  I2C1->CR1 &= ~(I2C_CR1_ACK);
  I2C1->CR1 |= (I2C_CR1_ACK);


  // 7-bit Addressing Mode
  I2C1->OAR1 &= ~(I2C_OAR1_ADDMODE);

  // Set a random address
  I2C1->OAR1 &= ~(I2C_OAR1_ADD1_7);
  I2C1->OAR1 |= (0x4000|0x33);

  // Set bit 14 to 0x1. From reference manual
  //I2C1->OAR1 |= (1U << 14U);

  // Enable the selected I2C peripheral
  I2C1->CR1 |= I2C_CR1_PE;

}


static void CS43L22_I2S_init(){

  /* For reference
  uint16_t pllm = GetPLLM();
  uint16_t plli2sn = GetPLLI2SN();
  uint16_t plli2sr = GetPLLI2SR();

  // I2SxCLK is the the clock sent to the I2S Clock Generator to produce all
  // of the clock signals
  uint32_t I2SxCLK = ((HSE_VALUE/pllm)*plli2sn)/plli2sr;
  */


  // Enable the periphery clock for SPI/I2S interface
  RCC->APB1ENR |= (RCC_APB1ENR_SPI3EN);

  // I2SE: I2S Enable
  //    0: I2S peripheral is disabled <----
  //    1: I2S peripheral is enabled
  // Note: This bit is not used in SPI mode.
  SPI3->I2SCFGR &= ~(SPI_I2SCFGR_I2SE);

  // I2SMOD: I2S mode selection
  //    0: SPI mode is selected
  //    1: I2S mode is selected <----
  // Note: This bit should be configured when the SPI or I2S is disabled
  SPI3->I2SCFGR |= (SPI_I2SCFGR_I2SMOD);

  // Here enable the PLLI2S to be the source of the clock and set Prescaler
  SystemPLLI2SEnable(258,3);

  // Configure the some additional divider to achieve audio sampling clock
  // i2sdiv and i2s odd are derrived from table in refernce manual for
  // 48kHz, there is an equation that uses these factors to get close
  // to your desired audio sampling frequency;
  uint32_t i2sdiv = 3;
  uint32_t i2sodd = 1;

  // ODD: Odd factor for the prescaler
  //    0: real divider value is = I2SDIV *2
  //    1: real divider value is = (I2SDIV * 2)+1
  // Refer to Section 28.4.4 on page 910. Not used in SPI mode.
  // Note: This bit should be configured when the I2S is disabled.
  //       It is used only when the I2S is in master mode.
  SPI3->I2SPR &= ~(SPI_I2SPR_ODD);
  SPI3->I2SPR |= (i2sodd << SPI_I2SPR_ODD_Pos);

  // I2SDIV: I2S Linear prescaler
  //    I2SDIV [7:0] = 0 or I2SDIV [7:0] = 1 are forbidden values.
  // Refer to Section 28.4.4 on page 910. Not used in SPI mode.
  // Note: These bits should be configured when the I2S is disabled.
  //       It is used only when the I2S is in master mode.
  SPI3->I2SPR &= ~(SPI_I2SPR_I2SDIV);
  SPI3->I2SPR |= (i2sdiv << SPI_I2SPR_I2SDIV_Pos);

  // MCKOE: Master clock output enable
  //    0: Master clock output is disabled
  //    1: Master clock output is enabled <----
  // Note: This bit should be configured when the I2S is disabled.
  //       It is used only when the I 2 S is in master mode.
  //       This bit is not used in SPI mode
  SPI3->I2SPR |= SPI_I2SPR_MCKOE;

  // I2SCFG: I2S configuration mode
  //    00: Slave - transmit
  //    01: Slave - receive
  //    10: Master - transmit <----
  //    11: Master - receive
  // Note: This bit should be configured when the I2S is disabled.
  // It is not used in SPI mode.
  SPI3->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG);
  SPI3->I2SCFGR |= (SPI_I2SCFGR_I2SCFG_1);

  // I2SSTD: I2S standard selection
  //    00: I2S Philips standard. <----
  //    01: MSB justified standard (left justified)
  //    10: LSB justified standard (right justified)
  //    11: PCM standard
  // For more details on I2S standards, refer to Section 28.4.3 on page 904.
  // Not used in SPI mode.
  // Note: For correct operation, these bits should be configured when the
  //       I2S is disabled.
  SPI3->I2SCFGR &= ~(SPI_I2SCFGR_I2SSTD);

  // CKPOL: Steady state clock polarity
  //    0: I2S clock steady state is low level <----
  //    1: I2S clock steady state is high level
  // Note: For correct operation, this bit should be configured when the
  //       I2S is disabled.
  // This bit is not used in SPI mode
  SPI3->I2SCFGR &= ~(SPI_I2SCFGR_CKPOL);

  // DATLEN: Data length to be transferred
  //    00: 16-bit data length <----
  //    01: 24-bit data length
  //    10: 32-bit data length
  //    11: Not allowed
  // Note: For correct operation, these bits should be configured when the
  //       I2S is disabled.
  // This bit is not used in SPI mode.
  SPI3->I2SCFGR &= ~(SPI_I2SCFGR_DATLEN);

  // CHLEN: Channel length (number of bits per audio channel)
  //    0: 16-bit wide <----
  //    1: 32-bit wide
  // The bit write operation has a meaning only if DATLEN = 00
  // otherwise the channel length is fixed to 32-bit by hardware whatever the
  // value filled in. Not used in SPI mode.
  // Note: For correct operation, this bit should be configured when the
  //       I2S is disabled.
  SPI3->I2SCFGR &= ~(SPI_I2SCFGR_CHLEN);

  // I2SE: I2S Enable
  //    0: I2S peripheral is disabled
  //    1: I2S peripheral is enabled <----
  // Note: This bit is not used in SPI mode.
  SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;


}

// RESET Pin to LOW
void CS43L22_reset(){

  GPIOD->BSRR = GPIO_BSRR_BR4;

}

// RESET Pin to HIGH
void CS43L22_activate(){

  GPIOD->BSRR = GPIO_BSRR_BS4;

}


void CS43L22_writeReg(uint8_t address, uint8_t data){

  // START: Start generation
  // This bit is set and cleared by software and cleared by hardware
  // when start is sent or PE=0.
  // In Master Mode:
  //    0: No Start generation
  //    1: Repeated start generation
  // In Slave mode:
  //    0: No Start generation
  //    1: Start generation when the bus is free
  I2C1->CR1 |= I2C_CR1_START;

  // SB: Start bit (Master mode)
  //    0: No Start condition
  //    1: Start condition generated.
  // – Set when a Start condition generated.
  // – Cleared by software by reading the SR1 register followed by
  // writing the DR register, or by hardware when PE=0
  while(!(I2C1->SR1 & I2C_SR1_SB)){
    // Wait for start condition to be generated
  }

  // DR[7:0] 8-bit data register
  // Byte received or to be transmitted to the bus.
  //  – Transmitter mode: Byte transmission starts automatically when a byte
  //    is written in the DR register. A continuous transmit stream can be
  //    maintained if the next data to be transmitted is put in DR once the
  //    transmission is started (TxE=1)
  // – Receiver mode: Received byte is copied into DR (RxNE=1). A continuous
  //   transmit stream can be maintained if DR is read before
  //   the next data byte is received (RxNE=1).
  // Note: In slave mode, the address is not copied into DR.
  // Write collision is not managed (DR can be written if TxE=0).
  // If an ARLO event occurs on ACK pulse, the received byte is not copied into DR
  // and so cannot be read.
  I2C1->DR = CS43L22_address; // Send the chip address

  // ADDR: Address sent (master mode)
  // Address sent (Master)
  //  0: No end of address transmission
  //  1: End of address transmission
  // – For 10-bit addressing, the bit is set after the ACK of the 2nd byte.
  // – For 7-bit addressing, the bit is set after the ACK of the byte.
  // Note: ADDR is not set after a NACK reception
  while (!(I2C1 ->SR1 & I2C_SR1_ADDR )) {}// Wait for master transmitter mode.
	I2C1 ->SR2; // Must read SR2 to clear ADDR bit

  // TxE=1 , shift register empty, data register empty, write Data1 in DR
  while(!(I2C1->SR1 & I2C_SR1_TXE)){}

  I2C1->DR = address; // Transmit the address to write to.
	while(!(I2C1->SR1 & I2C_SR1_TXE)){} // Wait for byte to move to shift register.


  I2C1->DR = data; // Transmit the data of the to write to.

  // After the last tranmission of data
  while (!(I2C1 ->SR1 & I2C_SR1_BTF )){} // Wait for all bytes to finish.

	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.

}

// Reading a register is done by doing a write register sequence then
// doing a read sequence.
uint8_t CS43L22_readReg(uint8_t address){

  // Send the START generation
  I2C1->CR1 |= I2C_CR1_START;

  while(!(I2C1->SR1 & I2C_SR1_SB)){
    // Wait for start condition to be generated
  }

  I2C1->DR = CS43L22_address; // Send the chip address
  while (!(I2C1 ->SR1 & I2C_SR1_ADDR )) {}// Wait for master transmitter mode.
	I2C1 ->SR2; // Must read SR2 to clear ADDR bit

  // TxE=1 , shift register empty, data register empty, write Data1 in DR
  while(!(I2C1->SR1 & I2C_SR1_TXE)){}

  I2C1->DR = address; // Transmit the address to write to.
  // After the last tranmission of data
  while (!(I2C1 ->SR1 & I2C_SR1_BTF )){} // Wait for all bytes to finish.
	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.

  // Read Sequence
  I2C1->CR1 |= I2C_CR1_START;
   while(!(I2C1->SR1 & I2C_SR1_SB)){
    // Wait for start condition to be generated
  }

  I2C1->DR = (CS43L22_address|0x1); // Send the chip address
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


void CS43L22_powerUp(void){

 // Reset the Codec
  CS43L22_reset();

  // Wait till the power supplies are ready
  Delay(10);

  // Bring RESET Pin to high so we can now program the part
  CS43L22_activate();

  // Keep the device powered down while we program our desired settings
  CS43L22_writeReg(0x02,0x01);

  // Headphone Always ON, Speakers OFF
  CS43L22_writeReg(0x04,0xAF);

  // Automatic Clock Control Detection
  CS43L22_writeReg(0x05,0x81);

  // I2S mode
  CS43L22_writeReg(0x06,0x04);

  // Set volume
  CS43L22_writeReg(0x20, (0xff + 0x19) & 0xff);
	CS43L22_writeReg(0x21, (0xff + 0x19) & 0xff);

  // Power on the codec.
	CS43L22_writeReg(0x02, 0x9E);

	// Configure codec for fast shutdown.
	CS43L22_writeReg(0x0A, 0x00); // Disable the analog soft ramp.
	CS43L22_writeReg(0x0E, 0x04); // Disable the digital soft ramp.

  // Initialization Sequence (p. 32)
  CS43L22_writeReg(0x00,0x99);
  CS43L22_writeReg(0x47,0x80);
  uint8_t value=CS43L22_readReg(0x32);
  CS43L22_writeReg(0x32,(value|0x80));
  CS43L22_writeReg(0x32,(value&(~0x80)));
  CS43L22_writeReg(0x00,0x00);

  // Disable the limiter attack level
  CS43L22_writeReg(0x27, 0x00);

  // Adjust Bass and Treble levels
  CS43L22_writeReg(0x1F, 0x0F);

  // Adjust PCM volume level
  CS43L22_writeReg(0x1A, 0x0A);
  CS43L22_writeReg(0x1B, 0x0A);

  CS43L22_writeReg(0x02,0x9E);

}


void CS43L22_powerDown(void){

  CS43L22_writeReg(0x0F,0xC0);
	CS43L22_writeReg(0x02, 0x9F);
  CS43L22_reset();

}
