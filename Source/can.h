#pragma once
#include "board.h"
#include "drv_system.h"
#include "uart.h"

#define PGN_DEV_RESET					((uint8_t) 0x3E)
#define PGN_READ_FLASH				((uint8_t) 0x01)
#define PGN_ErasePage_FLASH		((uint8_t) 0x02)
#define PGN_WRITE_FLASH				((uint8_t) 0x03)
#define PGN_i2cRead						((uint8_t) 0x04)
#define PGN_i2cWrite					((uint8_t) 0x05)
#define PGN_readReg						((uint8_t) 0xee)


typedef struct
{
	union
	{
		struct
		{
			uint8_t sa :8;
			uint8_t	ps :8;
			uint8_t pf :8;
			uint8_t dp :1;
			uint8_t r  :1;
			uint8_t p  :3;
		};
		uint32_t	idt;
	};
	uint8_t len;
	union
	{
		uint8_t data[8] ;
		uint32_t data_u32[2];
	};
	
}j1939msg_t;

void canInit (void);
void RXcan (uint32_t *data);
