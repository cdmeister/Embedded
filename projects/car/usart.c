#include "usart.h"



void USARTx_Init(USART_TypeDef * USARTx){


  /* PA2 (Tx) PA3 (Rx)*/
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  GPIOA->MODER &= ~(0xF<<4); /* Clear mode bits */
  GPIOA->MODER |= 0xA<<4;/* PA2 and PA3  are on GPIOA */

  GPIOA->AFR[0] = (0x77 << 8); /*Set alternate function for Pins 2 and 3*/

  GPIOA->OTYPER &= ~(0x3<<2); /*Configure as output open-drain */

  GPIOA->OSPEEDR &=~(0xF<<4); /* Configure as high speed */
  GPIOA->OSPEEDR |= (0xF<<4);

  GPIOA->PUPDR &= ~(0xF<<4); /*no pul-up, no pull-down*/
  GPIOA->PUPDR |= (0x5<<4); /*no pul-up, no pull-down*/


  /* Enable USART2*/
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

   /* Disable USART*/
  USARTx->CR1 &=~(0x1U << 13);
  USARTx->CR1 &=~USART_CR1_UE;

  /* Set data Length to 8 bits
   00 = 8 bits  01 = 9 bits
   10 = 7 bits
  */
  USARTx->CR1 &= ~(0x1U << 12);
  USARTx->CR1 &=~USART_CR1_M;

   /*Select 1 stop bit( bit 13 abd bit 12)
   00 = 1 stop bits   01 = 0.5 stop bit
   10 = 2 stop bits   11 = 1.5 stop bits
  */
  /*USARTx-> CR2 &= ~(0x3U << 12);*/
  USARTx->CR2 &= ~USART_CR2_STOP;

   /*Set parity control as no parity
   0 = no parity,
   1 = parity enabled ( then, prgram PS bit to select Even or Odd parity
  */
  USARTx->CR1 &= ~USART_CR1_PCE;

   /*Oversampling by 16
   0 = oversampling by 16, 1 = oversampling by 8
  */
  USARTx->CR1 &= ~USART_CR1_OVER8;

   /*Set Baud Rate to 9600 using APB Frequency
   Max frequency for USART2 (42 MHz)
   TODO: create function to calculate the value instead of
         hardcoding
  */
  USARTx->BRR = 0x1117;

   /*Enable transmission and reception*/
  USARTx->CR1 |= ( USART_CR1_RE| USART_CR1_TE );

   /*Receive register not empty interrupt*/
  /*USARTx->CR1 |= USART_CR1_RXNEIE;*/

   /*Transmit register empty interrupt*/
  /*USARTx->CR1 &= ~ USART_CR1_TXEIE;*/

   /*Set the priority of the interupt*/
  NVIC_SetPriority(USART2_IRQn, 1);

   /*Enable NVIC interrupt*/
  NVIC_EnableIRQ(USART2_IRQn);

  /* Enable USART*/
  USARTx->CR1 |=USART_CR1_UE;


}
void USART_printnum(USART_TypeDef * USARTx, uint16_t num){


  char str[18];
  char * s = str + sizeof(str);
  --s;
  *s = '\0';
  --s;
  *s = '\n';
  --s;
  *s = '\r';



  do {
    --s;
    *s = (char)(num%10) + '0';
    num /= 10;
  }
  while(num);



  USART_print(USARTx,s);
}
void USART_print(USART_TypeDef * USARTx, const char * fmt){

  while(*fmt){

    /* First check if the Transmission Data register is empty*/
    while ( !(USARTx->SR & USART_SR_TXE)){};

    /* Write Data to the register which will then get shifted out*/
    USARTx->DR =(*fmt);
    ++fmt;
  }

  /* Wait until transmission is complete*/
  while ( !(USARTx->SR & USART_SR_TC)){};

}
