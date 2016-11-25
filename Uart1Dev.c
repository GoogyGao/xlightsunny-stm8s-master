#include "Uart2Dev.h"
#include "stm8s.h"

uint8_t USART1_ReceiveDataBuf[20];
uint8_t Buff_Cnt ;
uint8_t USART1_FLAG ;

void uart2_config(void)
{
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART2, ENABLE);
  
  UART2_DeInit();
  
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_5, ENABLE);//TX
  GPIO_ExternalPullUpConfig(GPIOD, GPIO_PIN_6, ENABLE);//RX
  
  
  UART2_Init(115200, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
  
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
    if (pBuf[ucPos] != '\0') {
      Uart2SendByte(pBuf[ucPos ++]);
    } else 
      break;
  } while (1);
  
  return 1;


}



