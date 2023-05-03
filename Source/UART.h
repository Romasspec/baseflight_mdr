#pragma once
#include "MDR32F9Qx_uart.h"
#include "drv_system.h"

void uartInit (void);
void uartTransmit (uint8_t* data, uint16_t len);
