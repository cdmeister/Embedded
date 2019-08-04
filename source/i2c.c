#include "i2c.h"


inline void I2C_clock_init(I2C_TypeDef * I2Cx){

  // Enable I2C Periphal bus
  if(I2Cx == I2C1) RCC->APB1ENR |= (RCC_APB1ENR_I2C1EN);
  else if(I2Cx == I2C2) RCC->APB1ENR |= (RCC_APB1ENR_I2C2EN);
  else if(I2Cx == I2C3) RCC->APB1ENR |= (RCC_APB1ENR_I2C3EN);
  else {
    // Todo: Figure out what to do for error handling
    while(1);
  }


  // Disable I2C for configuration(Espicialy for TRISE)
  I2Cx->CR1 &= ~(I2C_CR1_PE);

 // Enable the selected I2C peripheral
  I2Cx->CR1 |= I2C_CR1_PE;

}

inline void I2C_CR1_mode(I2C_TypeDef * I2Cx){

}

inline void I2C_OAR_init(I2C_TypeDef * I2Cx){


}
