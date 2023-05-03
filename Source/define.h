#ifndef DEFINE_H
#define DEFINE_H

#include <stdint.h>

#define PGN_DEV_RESET					((uint8_t) 0x3E)
#define PGN_DEV_RUN_FD				((uint8_t) 0x01)
#define PGN_DEV_RUN_BC				((uint8_t) 0x02)
#define PGN_DEV_Uout					((uint8_t) 0x03)
#define PGN_DEV_kp						((uint8_t) 0x04)
#define PGN_DEV_ki						((uint8_t) 0x05)
#define PGN_DEV_BUFF_OUT			((uint8_t) 0x06)

#define PB0										0
#define PB1										1
#define PB2										2
#define PD6										6
#define PD7										7
#define A_HIGH 								(MDR_PORTB->RXTX |= (1<<PB0))
#define A_LOW 								(MDR_PORTB->RXTX &=~(1<<PB0))

#define B_HIGH 								(MDR_PORTB->RXTX |= (1<<PB1))
#define B_LOW 								(MDR_PORTB->RXTX &=~(1<<PB1))

#define C_HIGH 								(MDR_PORTB->RXTX |= (1<<PB2))
#define C_LOW 								(MDR_PORTB->RXTX &=~(1<<PB2))

#define D_HIGH 								(MDR_PORTD->RXTX |= (1<<PD6))
#define D_LOW 								(MDR_PORTD->RXTX &=~(1<<PD6))

// flags bits
#define CCR_UPDATE_FD	0
#define CCR_UPDATE_BC	1
#define FORWARD				2
#define BUFF_OUT			3
#define UART_RX_MESS	4
#define START_UART_TX_MESS	5

#define BUFFSIZE			64
#define LENGHT_FRAME_RX_UART	6

#define START_BYTE       0x55
#define SPEED_FORWARD    0x01
#define SPEED_BACK       0x01
#define CCR_FORWARD      0x03
#define CCR_BACK         0x04



#define   _IQ24(A)      (long) ((A) * 16777216.0L)
#define   _IQ12(A)      (long) ((A) * 4096.0L)

//#define SINGLE_PHASE
#define DOUBLE_PHASE
#define F_CPU 80000000
#define F_NOM	((uint32_t) (120000))
#define F_MAX	((uint32_t) (200000))
#define F_MIN	((uint32_t) (50000))

#define T_MAX ((uint16_t) (F_CPU / F_MIN))				// 1600
#define T_MIN	((uint16_t) (F_CPU / F_MAX))				// 400

#define TIMER_ACT_ARR				((uint32_t) (800))
#define MAX_CCR							(TIMER_ACT_ARR - 20)

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

#endif
