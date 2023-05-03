#include "board.h"

static uartPort_t uartPort1;
static uartPort_t uartPort2;

// USART1 - Telemetry (RX/TX by DMA)
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode)
{
	uartPort_t *s;
	
	static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
	static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
	
	s = &uartPort1;
	s->port.vTable = uartVTable;
	s->port.baudRate = baudRate;
	
	s->port.rxBuffer = rx1Buffer;
	s->port.txBuffer = tx1Buffer;
	s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
	s->port.txBufferSize = UART1_TX_BUFFER_SIZE;
	
	return s;
}

// USART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode)
{	
	uartPort_t *s;
	static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
	static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];
	
	s = &uartPort2;
	s->port.vTable = uartVTable;
	s->port.baudRate = baudRate;
	
	s->port.rxBuffer = rx2Buffer;
	s->port.txBuffer = tx2Buffer;
	s->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
	s->port.txBufferSize = UART2_TX_BUFFER_SIZE;
	
	return s;
}

serialPort_t *uartOpen(MDR_UART_TypeDef *UARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode)
{
	UART_InitTypeDef UART_InitStructure;
	
	uartPort_t *s = NULL;
	
	if (!UARTx) return NULL;
	
	if (UARTx == MDR_UART1) s = serialUART1(baudRate, mode);
	if (UARTx == MDR_UART2) s = serialUART2(baudRate, mode);
	
	s->UARTx = UARTx;
	
	// common serial initialisation code should move to serialPort::init()
	s->port.rxBufferHead = s->port.rxBufferTail = 0;
	s->port.txBufferHead = s->port.txBufferTail = 0;
	
	// callback for IRQ-based RX ONLY
	s->port.callback = callback;
	s->port.mode = mode;
	s->port.baudRate = baudRate;
	s->rxDMAChannel = NULL;
	s->txDMAChannel = NULL;
	
	//UART_InitStructure.
	UART_InitStructure.UART_BaudRate = baudRate;
	UART_InitStructure.UART_WordLength = UART_WordLength8b;
	UART_InitStructure.UART_StopBits = UART_StopBits1;
	UART_InitStructure.UART_Parity = UART_Parity_No;
	UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
	UART_InitStructure.UART_FIFOMode = UART_FIFO_OFF;
	
	UART_Init(UARTx, &UART_InitStructure);
	UART_Cmd(UARTx, ENABLE);
	
	// Receive DMA or IRQ
    if (mode & MODE_RX) {
        if (s->rxDMAChannel) {
		
		} else {
			
            UART_ITConfig(UARTx, UART_IT_RX, ENABLE);
			if (UARTx == MDR_UART1) {
				NVIC_EnableIRQ (UART1_IRQn);
				NVIC_SetPriority(UART1_IRQn,7);
			} else if (UARTx == MDR_UART2) {
				NVIC_EnableIRQ (UART2_IRQn);
				NVIC_SetPriority(UART2_IRQn,7);
			}
			
        }
		
	}
	
	// Transmit DMA or IRQ
    if (mode & MODE_TX) {
        if (s->txDMAChannel) {
			
		} else {
			
            UART_ITConfig(UARTx, UART_IT_TX, ENABLE);
			if (UARTx == MDR_UART1) {
				NVIC_EnableIRQ (UART1_IRQn);
				NVIC_SetPriority(UART1_IRQn,7);
			} else if (UARTx == MDR_UART2) {
				NVIC_EnableIRQ (UART2_IRQn);
				NVIC_SetPriority(UART2_IRQn,7);
			}
        }
		
	}
	
	return (serialPort_t *)s;
}

const struct serialPortVTable uartVTable[] = {
	{
		uartWrite,
		uartTotalBytesWaiting,
		uartRead,
		uartSetBaudRate,
		isUartTransmitBufferEmpty		
	}	
};

bool isUartTransmitBufferEmpty(serialPort_t *instance)
{
	uartPort_t *s = (uartPort_t *)instance;
//    if (s->txDMAChannel)
//        return s->txDMAEmpty;
//    else
//        return s->port.txBufferTail == s->port.txBufferHead;
	return s->port.txBufferTail == s->port.txBufferHead;
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    UART_InitTypeDef UART_InitStructure;
    uartPort_t *s = (uartPort_t *)instance;

    UART_InitStructure.UART_BaudRate = baudRate;
    UART_InitStructure.UART_WordLength = UART_WordLength8b;
    UART_InitStructure.UART_StopBits = UART_StopBits1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_FIFOMode = UART_FIFO_OFF;
    
    UART_Init(s->UARTx, &UART_InitStructure);

    s->port.baudRate = baudRate;
}

uint8_t uartRead(serialPort_t *instance)
{
	uint8_t ch;
	uartPort_t *s = (uartPort_t *)instance;

//    if (s->rxDMAChannel) {
//        ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
//        if (--s->rxDMAPos == 0)
//            s->rxDMAPos = s->port.rxBufferSize;
//    } else {
//        ch = s->port.rxBuffer[s->port.rxBufferTail];
//        s->port.rxBufferTail = (s->port.rxBufferTail + 1) % s->port.rxBufferSize;
//    }
	
	ch = s->port.rxBuffer[s->port.rxBufferTail];
	s->port.rxBufferTail = (s->port.rxBufferTail + 1) % s->port.rxBufferSize;

	return ch;
}

uint8_t uartTotalBytesWaiting(serialPort_t *instance)
{
	uartPort_t *s = (uartPort_t *)instance;
    // FIXME always returns 1 or 0, not the amount of bytes waiting
//    if (s->rxDMAChannel)
//        return s->rxDMAChannel->CNDTR != s->rxDMAPos;
//    else
//        return s->port.rxBufferTail != s->port.rxBufferHead;
	
	return s->port.rxBufferTail != s->port.rxBufferHead;
}

void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.txBuffer[s->port.txBufferHead] = ch;
    s->port.txBufferHead = (s->port.txBufferHead + 1) % s->port.txBufferSize;

//    if (s->txDMAChannel) {
//        if (!(s->txDMAChannel->CCR & 1))
//            uartStartTxDMA(s);
//    } else {
//        USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);		
//    }
	
	UART_ITConfig(s->UARTx, UART_IT_TX, ENABLE);
}

// USART1 Rx/Tx IRQ Handler
void UART1_IRQHandler (void)
{
	uartPort_t *s = &uartPort1;
    uint16_t SR = s->UARTx->FR;
	
	if (SR & UART_FR_RXFF) {
        // If we registered a callback, pass crap there
		if (s->port.callback) {
			s->port.callback (s->UARTx->DR);
		} else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->UARTx->DR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
	}
		
	if (SR & UART_FR_TXFE) {
		if (s->port.txBufferTail != s->port.txBufferHead) {
            s->UARTx->DR = s->port.txBuffer[s->port.txBufferTail];
			s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
		} else {
			UART_ITConfig (s->UARTx, UART_IT_TX, DISABLE);
		}
	}		
}

// USART2 Rx/Tx IRQ Handler
void UART2_IRQHandler (void)
{
	uartPort_t *s = &uartPort2;
    uint16_t SR = s->UARTx->FR;
	
	if (SR & UART_FR_RXFF) {
        // If we registered a callback, pass crap there
		if (s->port.callback) {
			s->port.callback (s->UARTx->DR);
		} else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->UARTx->DR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
	}
	
	if (SR & UART_FR_TXFE) {
		if (s->port.txBufferTail != s->port.txBufferHead) {
            s->UARTx->DR = s->port.txBuffer[s->port.txBufferTail];
			s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
		} else {
			UART_ITConfig (s->UARTx, UART_IT_TX, DISABLE);
		}
	}	
}

