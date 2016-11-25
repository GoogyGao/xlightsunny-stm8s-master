#include "Uart2Dev.h"
#include "stm8s.h"


void uart2_config(void)
{
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
  
  UART2_DeInit();
  
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_5, ENABLE);//TX
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_6, ENABLE);//RX
  
  
  UART2_Init(9600, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
  
  UART2_ITConfig(UART2_IT_RXNE,ENABLE);
  
  UART2_Cmd(ENABLE);

  /* Enable general interrupts */
    enableInterrupts(); 
}


void Uart2SendByte(uint8_t data)
{
  // Usart1_SendData8 ((unsigned char) data);
  UART2_SendData8( data);
  
  // Loop until the end of tranmission
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
}



uint8_t Uart2SendString(uint8_t *pBuf)
{
    unsigned char ucPos;
  if (!pBuf)
    return 0;
  
  ucPos = 0;
  do {
    if (pBuf[ucPos] != '\n') {
      Uart2SendByte(pBuf[ucPos ++]);
    } else 
    {
      Uart2SendByte(pBuf[ucPos]);
      break;
    }
      
  } while (1);
  
  return 1;


}



