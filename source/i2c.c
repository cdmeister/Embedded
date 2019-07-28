#include "i2c.h"


inline void I2C_clock_init(I2C_TypeDef * I2Cx){

  // Enable I2C Periphal bus
  if(I2Cx == I2C1) RCC->AHB1ENR |= (RCC_AHB1ENR_I2C1EN);
  else if(I2Cx == I2C2) RCC->AHB1ENR |= (RCC_AHB1ENR_I2C2EN);
  else if(I2Cx == I2C3) RCC->AHB1ENR |= (RCC_AHB1ENR_I2C3EN);
  else {
    // Todo: Figure out what to do for error handling
    while(1);
  }


  // Disable I2C for configuration(Espicialy for TRISE)
  I2Cx->CR1 &= ~(I2C_CR1_PE);

  //  For now just hard code values and figure out how to
  //  make this more modular

  // Peripheral clock frequncy
  // This value must match the AHB frequency sincd I2C is conntect to AHB1 Bus
  I2Cx->CR2 &= ~(I2C_CR2_FREQ);
  I2Cx->CR2 |= (I2C_CR2_FREQ_5|I2C_CR2_FREQ_3|I2C_CR2_FREQ_1);

  // Pick Standard or Fast mode (Master Mode Selection)
  I2Cx->CCR &= (I2C_CCR_FS);

  // IC2 Duty Cycle (Only for Fast Mode)
  I2Cx->CCR &= (I2C_CCR_DUTY);

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
  I2Cx->CCR &= ~(I2C_CCR_CCR);
  I2Cx->CCR |= result;

  // Set Maximum Rise Time for standard mode
  I2Cx->TRISE = (pclk1/100000) + 1;

  // Enable the selected I2C peripheral
  I2Cx->CR1 |= I2C_CR1_PE;

}

inline void I2C_CR1_mode(I2C_TypeDef * I2Cx){

  // Select Mode I2C mode
  I2Cx->CR1 &= ~(I2C_CR1_SMBUS);

  // Enable I2C to generate ACK
  I2Cx-CR1 &= ~(I2C_CR1_ACK);
  I2Cx-CR1 |= (I2C_CR1_ACK);

}

inline void I2C_OAR_init(I2C_TypeDef * I2Cx){

  // 7-bit Addressing Mode
  I2Cx->OAR1 &= ~(I2C_OAR1_ADDMODE);

  // Set a random address
  I2Cx->OAR1 &= ~(I2C_OAR1_ADD1_7);

  // Set bit 14 to 0x1. From reference manual
  I2Cx->OAR1 |= (1U << 14U);


}
