#include "uart.h"

void uartInit (void)
{
	UART_BRGInit (MDR_UART2, UART_HCLKdiv1);
	MDR_UART2->IBRD = 43;
	MDR_UART2->FBRD = 26;
	MDR_UART2->CR = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
	MDR_UART2->LCR_H = UART_WordLength8b;
	UART_Cmd (MDR_UART2, ENABLE);
}

void uartTransmit (uint8_t* data, uint16_t len)
{
	uint32_t prevtime = micros();
		
	for(uint16_t i=0; i < len; i++)
	{
		while (micros() - prevtime < 20000) {}		
		prevtime = micros();
//		while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE) == RESET);
		UART_SendData (MDR_UART2, *(data+i));		
	}	
}
