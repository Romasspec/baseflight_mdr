#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_can.h>
#include <MDR32F9Qx_timer.h>
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_uart.h>
#include "define.h"
#include <string.h>

void RST_clk_Init (void);
void myPort_Init (void);
void CAN2_Init (void);
void myTimer1_Init (void);
void myTimer2_Init (void);
void myTimer3_Init (void);
void (*task_m)(void);
void task_m1 (void);
void task_m2 (void);
void set_Fnom (void);
void myADC_Init(void);
void myUART_Init(void);

CAN_RxMsgTypeDef RxMsg;
CAN_TxMsgTypeDef TxMsg;
j1939msg_t CANmsg;
uint16_t Tim_CCR, temp_ARR, timer_1ms[3], ADC_Result;
uint8_t	faza_shag, flags, VCNTTimer;
uint32_t Uout;
uint32_t UoutRef;
int32_t ErrorU, SumErrorU;
uint32_t PIDout;
int32_t kp;
int32_t ki;
int32_t kd;
int64_t temp;
uint16_t Cnt_Actuator, Cnt_interrupt;
uint32_t status;

float skor, skor_ref, skor_err, skor_PID, tik, skor_sum_err, Skor_ki;
typedef union{
	float var_f;
	uint32_t var_32;
}var;

var var1;

float Skor_kp;
volatile uint8_t cnt_speed_0;
float ACoef[3], BCoef[3], x[3], y[3];
uint8_t RX_BUF[BUFFSIZE], state_RX, RX_BUF_DATA[LENGHT_FRAME_RX_UART-1];


uint16_t enc_a, enc_b;
uint16_t buf[720];
uint16_t *bufpr, *bufpr_end, *bufpr_rend;
uint16_t timer1;
uint8_t sch;

int main (void)
{
//	CAN_RxMsgTypeDef RxMsg;
////	CAN_TxMsgTypeDef TxMsg;
//	j1939msg_t CANmsg;
//	uint32_t cnt_shag;
//	uint8_t	faza_shag, timer_1ms[2], flags;
	
	RST_clk_Init();
	myPort_Init();
	CAN2_Init ();
	myADC_Init ();
	myTimer1_Init ();
	myTimer2_Init ();
	myTimer3_Init ();
	myUART_Init();
	
	faza_shag = 0;
	VCNTTimer = 0;
	timer_1ms[0] = 0;
	timer_1ms[1] = 0;
	timer_1ms[2] = 0;
		
	UoutRef = _IQ24(0.415482);
	kp = _IQ24(0.1);
	ki = _IQ24(0.00390625);
	SumErrorU = 0;
	Cnt_Actuator = 0;
	Cnt_interrupt = 0;
	skor_ref = 1;
	Skor_kp = 1;
	Skor_ki = 0.015;
	skor_sum_err = 0;
	var1.var_f = 0.15f;
	enc_a = enc_b = 0xFFFF;
	

// 100 Гц
	ACoef[0] = 0.06745527606901530200;
	ACoef[1] = 0.13491055213803060000;
	ACoef[2] = 0.06745527606901530200;
	
	BCoef[0] = 1;
	BCoef[1] = -1.14298050253990090000;
	BCoef[2] = 0.41280159809618860000;
	
// 10 Гц	
//	ACoef[0] = 0.00094547653094439164;
//	ACoef[1] = 0.00189095306188878330;
//	ACoef[2] = 0.00094547653094439164;
//	
//	BCoef[0] = 1;
//	BCoef[1] = -1.91119706742607360000;
//	BCoef[2] = 0.91497583480143418000;	

// 1 Гц
//		ACoef[0] = 0.00000994703743556282;
//    ACoef[1] = 0.00001989407487112564;
//    ACoef[2] = 0.00000994703743556282;
//		
//		BCoef[0] = 1;
//		BCoef[1] = -1.99111429220165360000;
//    BCoef[2] = 0.99115359586893537000;
	
		x[2] = x[1] = x[0] = 0;
		y[2] = y[1] = y[0] = 0;
	
	task_m = &task_m1;
	bufpr = &buf[0];
	bufpr_end = &buf[720];
	bufpr_rend = &buf[10];
	timer1 = 100;
	
	while (1)
	{
//			if ((TIMER_GetFlagStatus (MDR_TIMER1, TIMER_STATUS_CNT_ARR)) == SET)
//			{
//				TIMER_ClearFlag (MDR_TIMER1, TIMER_STATUS_CNT_ARR);
//				timer_1ms[0] ++;
//				timer_1ms[1] ++;
//			}
		
		if (VCNTTimer > 20)
		{
				VCNTTimer = 0;
//				timer_1ms[0] ++;
//				timer_1ms[1] ++;
				MDR_PORTD->RXTX ^= (1<<PD7);
				MDR_PORTC->RXTX ^= (1<<1);
			
			if (cnt_speed_0 > 100)
			{
				enc_a = enc_b = 0xFFFD;
			}	
			else
			{
				cnt_speed_0 ++;
			}
		}
		
		(*task_m)();
	}
}

void task_m1 ()
{	
	if ((MDR_CAN2->BUF_CON[1] & CAN_STATUS_RX_FULL) ==  CAN_STATUS_RX_FULL)
			{
				MDR_CAN2->BUF_CON[1] &=~CAN_STATUS_RX_FULL;
								
				CAN_GetRawReceivedData (MDR_CAN2, 1, &RxMsg);
				CANmsg.idt = RxMsg.Rx_Header.ID;
				CANmsg.data_u32[1] = RxMsg.Data[1];
				CANmsg.data_u32[0] = RxMsg.Data[0];
				
				if (CANmsg.pf == PGN_DEV_RESET)
				{					
//					RST_CLK_PCLKcmd (RST_CLK_PCLK_WWDG, ENABLE);
					MDR_WWDG->CFR	= 0x00000180;
					MDR_WWDG->CR	 = 0x7F;
					MDR_WWDG->CR	|= 0x80;					
				}
				
				if (CANmsg.pf == PGN_DEV_RUN_FD)
				{
					Tim_CCR = (CANmsg.data[1] << 8 ) | CANmsg.data[0];
					if (Tim_CCR > MAX_CCR)
					{
						Tim_CCR = MAX_CCR;
					}
					flags |= (1<<CCR_UPDATE_FD);
				}
				
				else if (CANmsg.pf == PGN_DEV_RUN_BC)
				{						
					Tim_CCR = (CANmsg.data[1] << 8 ) | CANmsg.data[0];
					if (Tim_CCR > MAX_CCR)
					{
						Tim_CCR = MAX_CCR;
					}
					flags |= (1<<CCR_UPDATE_BC);
				}
				
				if (CANmsg.pf == PGN_DEV_Uout)						// 3
				{
					//UoutRef = CANmsg.data_u32[0];
					UoutRef = CANmsg.data_u32[0];
					skor_ref = (float) UoutRef / 10.0f;
					if (CANmsg.data_u32[1] == 0x00000001)
					{
						flags |= (1<<FORWARD);
					}
					else
					{
						flags &=~(1<<FORWARD);
					}
					//memcpy(&skor_ref, &UoutRef, 4);
					//skor_ref = (float) UoutRef;
					
				}
				
				if (CANmsg.pf == PGN_DEV_kp)							// 4
				{
					kp = CANmsg.data_u32[0];
					Skor_kp = (float) kp / 100.0f;
				}
				
				if (CANmsg.pf == PGN_DEV_ki)							// 5
				{
					ki = CANmsg.data_u32[0];
					Skor_ki = (float) ki / 1000.0f;
//					memcpy (&var1.var_f, &CANmsg.data_u32[0], 4);
//					var1.var_32 = CANmsg.data_u32[0];
				}
//				if (CANmsg.pf == PGN_DEV_BUFF_OUT)
//				{
//					flags |= (1<<BUFF_OUT);
//					bufpr = &buf[0];
//					CANmsg.p		= 7;
//					CANmsg.r		= 0;
//					CANmsg.dp		= 0;
//					CANmsg.pf 	= 0x01;
//					CANmsg.ps	 	= 0x01;
//					CANmsg.sa 	= 0x07;				
//					CANmsg.len	=	2;
//					TxMsg.ID 		= CANmsg.idt;
//					TxMsg.DLC		= CANmsg.len;
//					TxMsg.IDE		= CAN_ID_EXT;
//				}
			}
			
			if (timer_1ms[0] > 1000)
			{
				timer_1ms[0]	=	0;
								
//				ADC1_Start ();
//				MDR_PORTD->RXTX |= (1<<PD6);				
//				while (!ADC1_GetFlagStatus (ADC1_FLAG_END_OF_CONVERSION)) {MDR_PORTD->RXTX ^= (1<<PD7);}
//				MDR_PORTD->RXTX &=~(1<<PD6);
//				ADC_Result = (uint16_t) ADC1_GetResult ();				
				
				MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;				
				while ((MDR_ADC->ADC1_STATUS & ADC1_FLAG_END_OF_CONVERSION) == 0) {}							
				ADC_Result = (uint16_t) MDR_ADC->ADC1_RESULT;				
					
				CANmsg.p		= 7;
				CANmsg.r		= 0;
				CANmsg.dp		= 0;
				CANmsg.pf 	= 0x01;
				CANmsg.ps	 	= 0x00;
				CANmsg.sa 	= 0x07;
				
				CANmsg.len	=	8;
				
//				CANmsg.data[0]	= ADC_Result & 0xFF;
//				CANmsg.data[1]	= (ADC_Result >> 8) & 0xFF;
//				CANmsg.data[4]	= (int8_t) skor_ref;									//		//Cnt_Actuator & 0xFF;
//				CANmsg.data[5]	= 0x00;																//(Cnt_Actuator >> 8) & 0xFF;
//				CANmsg.data[6]	= ((uint16_t) skor) & 0xFF;								//Cnt_interrupt & 0xFF;
//				CANmsg.data[7]	= ((uint16_t) skor >> 8) & 0xFF;				//(Cnt_interrupt >> 8) & 0xFF;	
					
//					CANmsg.data_u32[1] =  var1;
//			***************************************************************************************************************************************************************************************************	
				TxMsg.ID 		= CANmsg.idt;
				TxMsg.DLC		= CANmsg.len;
				TxMsg.IDE		= CAN_ID_EXT;
//				TxMsg.Data[0]	= skor_err;// //CANmsg.data_u32[0];
//				TxMsg.Data[1] = (MDR_TIMER3->CCR2 << 16) | MDR_TIMER3->CCR1;

//				TxMsg.Data[0]	= (uint32_t) p_var1;
//				CANmsg.data[4] = *p_var1;
//				CANmsg.data[5] = *(p_var1+1);
//				CANmsg.data[6] = *(p_var1+2);
//				CANmsg.data[7] = *(p_var1+3);
//					CANmsg.data_u32[1] = *((uint32_t*) p_var1);

//				CANmsg.data[6] =
//				CANmsg.data[7] =
//				TxMsg.Data[1]	= var1;
//				var1 = var1 + 0.01;
//				TxMsg.Data[1]	= (uint8_t) (skor_PID * 100); //PIDout;
//				TxMsg.Data[0] = (uint32_t) (&var1.var_f);
				memcpy (&TxMsg.Data[0], &skor, 4);
				memcpy (&TxMsg.Data[1], &skor_err, 4);
//				var1.var_f = var1.var_f + 0.1f;
//				TxMsg.Data[1] = 
				CAN_Transmit (MDR_CAN2, 0, &TxMsg);
			}
			
//			if (timer_1ms[1] > 300)
//			{
//				timer_1ms[1]	=	0;
//				UART_SendData (MDR_UART2, sch++);
//			}
			
			if ((timer_1ms[1] > 20) && (flags & (1<<UART_RX_MESS)))
			{	
//				static uint8_t i = 0;
				timer_1ms[1]	=	0;
//				UART_SendData (MDR_UART2, RX_BUF_DATA[i]);
//				i++;
//				if (i == LENGHT_FRAME_RX_UART-1)
//				{
//					flags &=~(1<<UART_RX_MESS);
//					i = 0;
//				}
				if (RX_BUF_DATA[0] == 0x01)
				{
					UoutRef = RX_BUF_DATA[4]<<8 |RX_BUF_DATA[3];
					UoutRef = UoutRef<<8 |RX_BUF_DATA[2];
					UoutRef = UoutRef<<8 |RX_BUF_DATA[1];
					flags |= (1<<FORWARD);
					memcpy(&skor_ref, &UoutRef, 4);
//					skor_ref = (float) UoutRef;
				}
				else if (RX_BUF_DATA[0] == 0x02)
				{					
					UoutRef = RX_BUF_DATA[4]<<8 |RX_BUF_DATA[3];
					UoutRef = UoutRef<<8 |RX_BUF_DATA[2];
					UoutRef = UoutRef<<8 |RX_BUF_DATA[1];
					flags &=~(1<<FORWARD);
//					skor_ref = (float) UoutRef;
					memcpy(&skor_ref, &UoutRef, 4);
				}					
				else if (RX_BUF_DATA[0] == 0x03)
				{
					Tim_CCR = RX_BUF_DATA[2]<<8 |RX_BUF_DATA[1] ;
					if (Tim_CCR > MAX_CCR)
					{
						Tim_CCR = MAX_CCR;
					}
					flags |= (1<<CCR_UPDATE_FD);
				}	
				else if (RX_BUF_DATA[0] == 0x04)
				{
					Tim_CCR = RX_BUF_DATA[2]<<8 |RX_BUF_DATA[1] ;
					if (Tim_CCR > MAX_CCR)
					{
						Tim_CCR = MAX_CCR;
					}
					flags |= (1<<CCR_UPDATE_BC);
				}			
				else if (RX_BUF_DATA[0] == 0x05)
				{
					bufpr_rend = &bufpr[0] + (RX_BUF_DATA[2]<<8 |RX_BUF_DATA[1]);
					bufpr = &buf[0];
					flags |= (1<<BUFF_OUT);
				}
				else if (RX_BUF_DATA[0] == 0x06 )
				{
					flags ^= (1<<START_UART_TX_MESS);
				}
				else if (RX_BUF_DATA[0] == 0x07)
				{
					kp = RX_BUF_DATA[4]<<8 |RX_BUF_DATA[3];
					kp = kp<<8 |RX_BUF_DATA[2];
					kp = kp<<8 |RX_BUF_DATA[1];					
					memcpy(&Skor_kp, &kp, 4);
				}
				else if (RX_BUF_DATA[0] == 0x08)
				{
					ki = RX_BUF_DATA[4]<<8 |RX_BUF_DATA[3];
					ki = ki<<8 |RX_BUF_DATA[2];
					ki = ki<<8 |RX_BUF_DATA[1];					
					memcpy(&Skor_ki, &ki, 4);
				}

				flags &=~(1<<UART_RX_MESS);
			}
			
// отправляет байты раз в 20 мс
//			if ((timer_1ms[0] > timer1) && (flags & (1<<BUFF_OUT)))
//			{
//				union {	
//					
//					uint8_t message_b[4];
//					uint32_t message;
//					
//				}mes;
//				
//				static uint8_t i = 0;
//				
//				timer_1ms[0]	=	0;
//				
//				if ((bufpr < bufpr_end) && (i == 0))
//				{
//					UART_SendData (MDR_UART2, START_BYTE);
//					i = 1;
//				}
//				else if ((bufpr < bufpr_end) && (i < 5))
//				{
//					mes.message = *bufpr;
//					UART_SendData (MDR_UART2, mes.message_b[i-1]);
//					i++;
//				}
//				else if ((bufpr < bufpr_end) && (i > 4))
//				{
//					bufpr++;
//					i=0;
//				}
//				else
//				{
//					flags &=~(1<<BUFF_OUT);
//				}
//				
//			}
	
			if ((timer_1ms[0] > timer1) && (flags & (1<<BUFF_OUT)))
			{				
				union {						
					uint8_t message_b[4];
					uint32_t message;					
				}mes;
				
				uint8_t i;
				timer_1ms[0]	=	0;
								
				if(bufpr < bufpr_rend)
				{					
					mes.message = *bufpr;
					bufpr++;
					UART_SendData (MDR_UART2, START_BYTE);
					for (i = 0; i < 4; i++)
					{
						while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE) == RESET);
						UART_SendData (MDR_UART2, mes.message_b[i]);
					}
				}
				else
				{
					flags &=~(1<<BUFF_OUT);
					bufpr = &buf[0];
				}
			}
			
			if ((timer_1ms[2] > 100) && (flags & (1<<START_UART_TX_MESS)))
			{
				uint8_t i;
				union {						
					uint8_t message_b[4];
					uint32_t message;					
				}mes;
				memcpy(&mes.message, &skor, 4);
				//mes.message = enc_a;
				timer_1ms[2] = 0;
				UART_SendData (MDR_UART2, START_BYTE);
				for (i = 0; i < 4; i++)
					{
						while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_TXFE) == RESET);
						UART_SendData (MDR_UART2, mes.message_b[i]);
					}
				
			}
	
	task_m = &task_m2;
}

void task_m2 ()
{
	static uint8_t rxByte = 0;
	if (flags & (1<<CCR_UPDATE_FD))
	{
		flags &=~(1<<CCR_UPDATE_FD);
		//TIMER_SetChnCompare (MDR_TIMER3, TIMER_CHANNEL1, Tim_CCR);
		MDR_TIMER3->CCR1 = 400 + (Tim_CCR >> 1);
		MDR_TIMER3->CCR2 = 400 - (Tim_CCR >> 1);
	}
	
	if (flags & (1<<CCR_UPDATE_BC))
	{
		flags &=~(1<<CCR_UPDATE_BC);
		MDR_TIMER3->CCR1 = 400 - (Tim_CCR >> 1);
		MDR_TIMER3->CCR2 = 400 + (Tim_CCR >> 1);
	}
	
	if (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF) == SET)
	{		
		RX_BUF [rxByte] = UART_ReceiveData (MDR_UART2);
		
		if (rxByte == LENGHT_FRAME_RX_UART-1)
		{
			if (RX_BUF[0] == 0x55)
			{
				uint8_t i;
				for (i=1; i < LENGHT_FRAME_RX_UART; i++)
				{
					RX_BUF_DATA [i-1] = RX_BUF [i];
					flags |= (1<<UART_RX_MESS);
				}
			}
			rxByte = 0;
			state_RX = 0;
		}
		else
		{
			switch (state_RX)
			{
			case 0:
				if (RX_BUF[rxByte] == 0x55)
				{
					rxByte = 1;
					state_RX = 1;
				}				
			break;
			
			case 1:
				if (RX_BUF[rxByte] == 0xDA)
				{
					state_RX = 2;
				}
				else if (RX_BUF[rxByte] == 0x55)
				{
					rxByte = 1;
					state_RX = 1;
				}
				else
				{
					rxByte++;
				}
			break;
				
			case 2:
				if (RX_BUF[rxByte] == 0xDB)
				{
					RX_BUF[rxByte] = 0x55;					
				}
				else if (RX_BUF[rxByte] == 0xDC)
				{
					RX_BUF[rxByte] = 0xDA;
				}
				else if (RX_BUF[rxByte] == 0x55)
				{
					rxByte = 1;
					state_RX = 1;
				}
				else
				{
					rxByte = 0;
					state_RX = 0;
					break;
				}
				
				rxByte++;
				state_RX = 1;
			break;			
			}
		}		
	}
	
	task_m = &task_m1;
}
void RST_clk_Init()
{
	RST_CLK_DeInit();
	RST_CLK_HSEconfig (RST_CLK_HS_CONTROL_HSE_ON);								// eternal oscillator on
	while (!(RST_CLK_GetFlagStatus(RST_CLK_FLAG_HSERDY)));				// waiting
	RST_CLK_CPU_PLLcmd (ENABLE);																	// PLL on
	// configures the CPU_PLL clock source and multiplication factor
	RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSEdiv1, RST_CLK_CPU_PLLmul10);//RST_CLK_CPU_PLLsrcHSEdiv1
	while (!(RST_CLK_GetFlagStatus(RST_CLK_FLAG_PLLCPURDY)));
	// select the CPU_PLL output as input for CPU_C2_SEL
	RST_CLK_CPU_PLLuse (ENABLE);
	// configures the CPU_C3_SEL division factor
	RST_CLK_CPUclkPrescaler (RST_CLK_CPUclkDIV1);
	// select the HCLK clock source
	RST_CLK_CPUclkSelection (RST_CLK_CPUclkCPU_C3);
	// Enable peripheral clocks  
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_RST_CLK, ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_PORTA, 	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_PORTB, 	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_PORTC, 	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_PORTD, 	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_PORTE, 	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_PORTF, 	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_EEPROM,	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_TIMER1,	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_TIMER2,	ENABLE);
	RST_CLK_PCLKcmd ( RST_CLK_PCLK_TIMER3,	ENABLE);
	RST_CLK_PCLKcmd	( RST_CLK_PCLK_UART2, 	ENABLE);
	RST_CLK_PCLKcmd	( RST_CLK_PCLK_ADC,	  	ENABLE);
	RST_CLK_PCLKcmd (RST_CLK_PCLK_WWDG, 		ENABLE);	
}

void myPort_Init ()
{
	PORT_InitTypeDef Port_Initstructure;
	
	PORT_DeInit (MDR_PORTA);
	PORT_DeInit (MDR_PORTB);
	PORT_DeInit (MDR_PORTC);
	PORT_DeInit (MDR_PORTD);
	PORT_DeInit (MDR_PORTE);
	PORT_DeInit (MDR_PORTF);
	
	
	PORT_StructInit (&Port_Initstructure);
	// Configure PORTB pins  for output
	Port_Initstructure.PORT_Pin		= PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3;
	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
	PORT_Init (MDR_PORTB, &Port_Initstructure);
	
	// Configure PORTC pins  for output
	PORT_StructInit (&Port_Initstructure);
	Port_Initstructure.PORT_Pin		= PORT_Pin_1;
	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_PORT;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_SLOW;
	PORT_Init (MDR_PORTC, &Port_Initstructure);
	
	// Configure PORTD pins  for output
	PORT_StructInit (&Port_Initstructure);
	Port_Initstructure.PORT_Pin		= PORT_Pin_6 | PORT_Pin_7;
	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_PORT;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_SLOW;
	PORT_Init (MDR_PORTD, &Port_Initstructure);
	
	Port_Initstructure.PORT_Pin		= PORT_Pin_0;
	Port_Initstructure.PORT_OE		= PORT_OE_IN;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
	PORT_Init (MDR_PORTD, &Port_Initstructure);
	
	Port_Initstructure.PORT_Pin		= PORT_Pin_1;
	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
	PORT_Init (MDR_PORTD, &Port_Initstructure);
	
	// Configure PORTE pins  for input
	PORT_StructInit (&Port_Initstructure);
	Port_Initstructure.PORT_Pin		= PORT_Pin_0 | PORT_Pin_2;
//	Port_Initstructure.PORT_OE		= PORT_OE_IN;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
	PORT_Init (MDR_PORTE, &Port_Initstructure);
		 
	
	// Configure PORTE pins  for input (CAN2RX)
	PORT_StructInit (&Port_Initstructure);
	Port_Initstructure.PORT_Pin		= PORT_Pin_6;
//	Port_Initstructure.PORT_OE		= PORT_OE_IN;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	PORT_Init (MDR_PORTE, &Port_Initstructure);
	
	// Configure PORTE pins  for output (CAN2TX)
//	Port_Initstructure.PORT_PD		= PORT_PD_DRIVER;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
//	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
	Port_Initstructure.PORT_Pin		= PORT_Pin_7;
	PORT_Init (MDR_PORTE, &Port_Initstructure);
}

void CAN2_Init ()
{
	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	RST_CLK_PCLKcmd (RST_CLK_PCLK_CAN2, ENABLE);
	CAN_BRGInit (MDR_CAN2, CAN_HCLKdiv1);
	CAN_DeInit (MDR_CAN2);
	
	CAN_StructInit (&CAN_InitStructure);
	
	CAN_InitStructure.CAN_ROP			= DISABLE;
	CAN_InitStructure.CAN_SAP			= DISABLE;
	CAN_InitStructure.CAN_STM			= DISABLE;
	CAN_InitStructure.CAN_ROM			= DISABLE;
	CAN_InitStructure.CAN_OVER_ERROR_MAX = 255;
	CAN_InitStructure.CAN_SB			= CAN_SB_1_SAMPLE;
	CAN_InitStructure.CAN_PSEG		= CAN_PSEG_Mul_7TQ;
	CAN_InitStructure.CAN_SEG1		= CAN_SEG1_Mul_5TQ;
	CAN_InitStructure.CAN_SEG2		= CAN_SEG2_Mul_3TQ;
	CAN_InitStructure.CAN_SJW			= CAN_SJW_Mul_1TQ;
	CAN_InitStructure.CAN_BRP			= 19;
	CAN_Init (MDR_CAN2, &CAN_InitStructure);
	
	CAN_Cmd (MDR_CAN2, ENABLE);
	
	CAN_RxITConfig (MDR_CAN2, 1, ENABLE);
	CAN_TxITConfig (MDR_CAN2, 0, ENABLE);
	
	CAN_FilterInitStructure.Mask_ID =		 0x1F00FF00;
	CAN_FilterInitStructure.Filter_ID =	 0x0C000700;
	CAN_FilterInit (MDR_CAN2, 1, &CAN_FilterInitStructure);
	CAN_Receive (MDR_CAN2, 1, ENABLE);

}

void myTimer1_Init ()
{
	TIMER_CntInitTypeDef sTim_CntInit;	
	TIMER_CntStructInit (&sTim_CntInit);
	TIMER_DeInit (MDR_TIMER1);
	sTim_CntInit.TIMER_IniCounter 					= 0;
	sTim_CntInit.TIMER_Prescaler						= 9;
	sTim_CntInit.TIMER_Period								= 8000;
	sTim_CntInit.TIMER_CounterMode					= TIMER_CntMode_ClkFixedDir;
	sTim_CntInit.TIMER_CounterDirection			= TIMER_CntDir_Up;
	sTim_CntInit.TIMER_EventSource					= TIMER_EvSrc_None;
	sTim_CntInit.TIMER_FilterSampling				= TIMER_FDTS_TIMER_CLK_div_1;
	sTim_CntInit.TIMER_ARR_UpdateMode				= TIMER_ARR_Update_On_CNT_Overflow;
	
	TIMER_CntInit (MDR_TIMER1, &sTim_CntInit);
	TIMER_BRGInit (MDR_TIMER1, TIMER_HCLKdiv1);
	
	TIMER_ITConfig (MDR_TIMER1, TIMER_STATUS_CNT_ARR, ENABLE);
	NVIC_EnableIRQ (Timer1_IRQn);
	
	TIMER_Cmd (MDR_TIMER1, ENABLE);
}
void myTimer2_Init ()
{
	TIMER_CntInitTypeDef sTim_CntInit;	
	TIMER_ChnInitTypeDef sTim_ChnInit;
	
	TIMER_CntStructInit (&sTim_CntInit);
	TIMER_DeInit (MDR_TIMER2);
	sTim_CntInit.TIMER_IniCounter 					= 0;
	sTim_CntInit.TIMER_Prescaler						= 159;
	sTim_CntInit.TIMER_Period								= 0xFFFF;
	sTim_CntInit.TIMER_CounterMode					= TIMER_CntMode_ClkFixedDir;
	sTim_CntInit.TIMER_CounterDirection			= TIMER_CntDir_Up;
	sTim_CntInit.TIMER_EventSource					= TIMER_EvSrc_None;
	sTim_CntInit.TIMER_FilterSampling				= TIMER_FDTS_TIMER_CLK_div_4;
	sTim_CntInit.TIMER_ARR_UpdateMode				= TIMER_ARR_Update_On_CNT_Overflow;
	TIMER_CntInit (MDR_TIMER2, &sTim_CntInit);
	
	TIMER_ChnStructInit (&sTim_ChnInit);
	sTim_ChnInit.TIMER_CH_Number								= TIMER_CHANNEL1;
	sTim_ChnInit.TIMER_CH_Mode									= TIMER_CH_MODE_CAPTURE;
	sTim_ChnInit.TIMER_CH_ETR_Ena								= DISABLE;
	sTim_ChnInit.TIMER_CH_ETR_Reset							= TIMER_CH_ETR_RESET_Disable;
	sTim_ChnInit.TIMER_CH_BRK_Reset							= TIMER_CH_BRK_RESET_Disable;
//	sTim_ChnInit.TIMER_CH_REF_Format						= TIMER_CH_REF_Format6;
	sTim_ChnInit.TIMER_CH_Prescaler							= TIMER_CH_Prescaler_None;
	sTim_ChnInit.TIMER_CH_EventSource						= TIMER_CH_EvSrc_PE;
	sTim_ChnInit.TIMER_CH_FilterConf						= TIMER_Filter_8FF_at_FTDS_div_32;
	sTim_ChnInit.TIMER_CH_CCR_UpdateMode				= TIMER_CH_CCR_Update_On_CNT_eq_0;
	sTim_ChnInit.TIMER_CH_CCR1_Ena							= DISABLE;
	sTim_ChnInit.TIMER_CH_CCR1_EventSource			= TIMER_CH_CCR1EvSrc_NE;
	TIMER_ChnInit (MDR_TIMER2, &sTim_ChnInit);
	
	sTim_ChnInit.TIMER_CH_Number								= TIMER_CHANNEL3;
	TIMER_ChnInit (MDR_TIMER2, &sTim_ChnInit);

	TIMER_ITConfig (MDR_TIMER2, TIMER_STATUS_CCR_CAP_CH1, ENABLE);
	TIMER_ITConfig (MDR_TIMER2, TIMER_STATUS_CCR_CAP_CH3, ENABLE);
	MDR_TIMER2->STATUS = 0;
	NVIC_EnableIRQ (Timer2_IRQn);
	
	TIMER_BRGInit (MDR_TIMER2, TIMER_HCLKdiv1);
	TIMER_Cmd (MDR_TIMER2, ENABLE);
}
void myTimer3_Init ()
{
	TIMER_CntInitTypeDef 			sTim_CntInit;
	TIMER_ChnInitTypeDef			sTim_ChnInit;
	TIMER_ChnOutInitTypeDef		sTim_ChnOutInit;
	
	TIMER_CntStructInit (&sTim_CntInit);
	TIMER_DeInit (MDR_TIMER3);
	sTim_CntInit.TIMER_IniCounter 							= 0;
	sTim_CntInit.TIMER_Prescaler								= 4;
	sTim_CntInit.TIMER_Period										= TIMER_ACT_ARR;
	sTim_CntInit.TIMER_CounterMode							= TIMER_CntMode_ClkFixedDir;
	sTim_CntInit.TIMER_CounterDirection					= TIMER_CntDir_Up;
	sTim_CntInit.TIMER_EventSource							= TIMER_EvSrc_None;
	sTim_CntInit.TIMER_FilterSampling						= TIMER_FDTS_TIMER_CLK_div_1;
	sTim_CntInit.TIMER_ARR_UpdateMode						= TIMER_ARR_Update_On_CNT_Overflow;
//	sTim_CntInit.TIMER_ARR_UpdateMode						= TIMER_ARR_Update_Immediately;
	TIMER_CntInit (MDR_TIMER3, &sTim_CntInit);
	
	
	TIMER_ChnStructInit (&sTim_ChnInit);
	sTim_ChnInit.TIMER_CH_Number 								= TIMER_CHANNEL1;
	sTim_ChnInit.TIMER_CH_Mode									= TIMER_CH_MODE_PWM;
	sTim_ChnInit.TIMER_CH_ETR_Ena								= DISABLE;
	sTim_ChnInit.TIMER_CH_ETR_Reset							= TIMER_CH_ETR_RESET_Disable;
	sTim_ChnInit.TIMER_CH_BRK_Reset							= TIMER_CH_BRK_RESET_Disable;
	sTim_ChnInit.TIMER_CH_REF_Format						= TIMER_CH_REF_Format6;
	sTim_ChnInit.TIMER_CH_Prescaler							= TIMER_CH_Prescaler_None;
	sTim_ChnInit.TIMER_CH_EventSource						= TIMER_CH_EvSrc_PE;
	sTim_ChnInit.TIMER_CH_FilterConf						= TIMER_Filter_1FF_at_TIMER_CLK;
	sTim_ChnInit.TIMER_CH_CCR_UpdateMode				= TIMER_CH_CCR_Update_On_CNT_eq_0;
	sTim_ChnInit.TIMER_CH_CCR1_Ena							= DISABLE;
	sTim_ChnInit.TIMER_CH_CCR1_EventSource			= TIMER_CH_CCR1EvSrc_PE;
	TIMER_ChnInit (MDR_TIMER3, &sTim_ChnInit);
	
	
	TIMER_ChnOutStructInit (&sTim_ChnOutInit);
	sTim_ChnOutInit.TIMER_CH_Number							= TIMER_CHANNEL1;
	sTim_ChnOutInit.TIMER_CH_DirOut_Polarity		= TIMER_CHOPolarity_NonInverted;
	sTim_ChnOutInit.TIMER_CH_DirOut_Source			= TIMER_CH_OutSrc_REF;						//TIMER_CH_OutSrc_DTG;
	sTim_ChnOutInit.TIMER_CH_DirOut_Mode				= TIMER_CH_OutMode_Output;
	sTim_ChnOutInit.TIMER_CH_NegOut_Polarity		= TIMER_CHOPolarity_NonInverted;
	sTim_ChnOutInit.TIMER_CH_NegOut_Source			= TIMER_CH_OutSrc_DTG;
	sTim_ChnOutInit.TIMER_CH_NegOut_Mode				= TIMER_CH_OutMode_Input;					//TIMER_CH_OutMode_Output;
	sTim_ChnOutInit.TIMER_CH_DTG_MainPrescaler	= 40;
	sTim_ChnOutInit.TIMER_CH_DTG_AuxPrescaler		= 0;
	sTim_ChnOutInit.TIMER_CH_DTG_ClockSource		= TIMER_CH_DTG_ClkSrc_TIMER_CLK;
	TIMER_ChnOutInit (MDR_TIMER3, &sTim_ChnOutInit);
	
	
	sTim_ChnInit.TIMER_CH_Number 								= TIMER_CHANNEL2;
	TIMER_ChnInit (MDR_TIMER3, &sTim_ChnInit);
	
	
	sTim_ChnOutInit.TIMER_CH_Number							= TIMER_CHANNEL2;
	TIMER_ChnOutInit (MDR_TIMER3, &sTim_ChnOutInit);
	
	MDR_TIMER3->CCR1 = TIMER_ACT_ARR /2;
	MDR_TIMER3->CCR2 = TIMER_ACT_ARR /2;
	
	TIMER_BRGInit (MDR_TIMER3, TIMER_HCLKdiv1);
	TIMER_Cmd (MDR_TIMER3, ENABLE);
}
void myADC_Init()
{
	
	ADC_InitTypeDef		ADC_InitStructure;
	ADCx_InitTypeDef 	ADCx_InitStructure;
	
	ADC_StructInit (&ADC_InitStructure);
	ADC_DeInit ();
	ADC_InitStructure.ADC_SynchronousMode 				= ADC_SyncMode_Independent;
	ADC_InitStructure.ADC_StartDelay							= 0;
	ADC_InitStructure.ADC_TempSensor							= ADC_TEMP_SENSOR_Disable;
	ADC_InitStructure.ADC_TempSensorAmplifier			= ADC_TEMP_SENSOR_AMPLIFIER_Disable;
	ADC_InitStructure.ADC_TempSensorConversion		= ADC_TEMP_SENSOR_CONVERSION_Disable;
	ADC_InitStructure.ADC_IntVRefConversion				= ADC_VREF_CONVERSION_Disable;
	ADC_InitStructure.ADC_IntVRefTrimming					= 0;
	ADC_Init (&ADC_InitStructure);
	
	ADCx_StructInit (&ADCx_InitStructure);
	ADCx_InitStructure.ADC_ClockSource						= ADC_CLOCK_SOURCE_CPU;
	ADCx_InitStructure.ADC_SamplingMode						= ADC_SAMPLING_MODE_SINGLE_CONV;
	ADCx_InitStructure.ADC_ChannelSwitching				= ADC_CH_SWITCHING_Disable;
	ADCx_InitStructure.ADC_ChannelNumber					= ADC_CH_ADC4;
	ADCx_InitStructure.ADC_Channels								= 0;
	ADCx_InitStructure.ADC_LevelControl						= ADC_LEVEL_CONTROL_Disable;
	ADCx_InitStructure.ADC_LowLevel								= 0;
	ADCx_InitStructure.ADC_HighLevel							= 0;
	ADCx_InitStructure.ADC_VRefSource							= ADC_VREF_SOURCE_INTERNAL;
	ADCx_InitStructure.ADC_IntVRefSource					= ADC_INT_VREF_SOURCE_INEXACT;
	ADCx_InitStructure.ADC_Prescaler							= ADC_CLK_div_8;
	ADCx_InitStructure.ADC_DelayGo								= 0;
	ADC1_Init (&ADCx_InitStructure);
	
	ADC1_Cmd (ENABLE);	
}
void myUART_Init ()
{	
//	UART_InitTypeDef UART_InitStructure;
	
//	UART_DeInit (MDR_UART2);
	UART_BRGInit (MDR_UART2, UART_HCLKdiv1);
//	UART_StructInit (&UART_InitStructure);
//	UART_InitStructure.UART_BaudRate = 9600;
//	UART_InitStructure.UART_WordLength = UART_WordLength8b;
//	UART_InitStructure.UART_StopBits = UART_StopBits1;
//	UART_InitStructure.UART_Parity = UART_Parity_No;
//	UART_InitStructure.UART_FIFOMode = UART_FIFO_OFF;
//	UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
//	UART_Init (MDR_UART2, &UART_InitStructure);
	MDR_UART2->IBRD = 43;
	MDR_UART2->FBRD = 26;
	MDR_UART2->CR = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
	MDR_UART2->LCR_H = UART_WordLength8b;
	UART_Cmd (MDR_UART2, ENABLE);
	
}
void Timer1_IRQHandler ()
{	
	timer_1ms[0] ++;
	timer_1ms[1] ++;
	timer_1ms[2] ++;
	
	if (MDR_TIMER1->STATUS & TIMER_STATUS_CNT_ARR)
	{
		MDR_TIMER1->STATUS &=~TIMER_STATUS_CNT_ARR;
		VCNTTimer++;
//		MDR_PORTD->RXTX ^= (1<<PD6);
//		MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;				
//		while ((MDR_ADC->ADC1_STATUS & ADC1_FLAG_END_OF_CONVERSION) == 0) {}							
//		Uout =  MDR_ADC->ADC1_RESULT & 0x0FFF;
//		
//		ErrorU = (UoutRef - (Uout << 12)) ;																	// q24-(q12<<12)=q24		
//		SumErrorU = (int32_t) (SumErrorU + (((int64_t) ki * ErrorU) >> 24));						// q24+((q24*q24)>>24) = q24
//		if (SumErrorU > _IQ24 (1))
//		{
//			SumErrorU = _IQ24 (1);
//		}
//		if (SumErrorU < _IQ24 (-1))
//		{
//			SumErrorU = _IQ24 (-1);
//		}
//		temp = (((int64_t) kp * ErrorU) >> 24) + SumErrorU;						// ((q24*q24)>>24)+q24 = q24			
//		
//		if (temp < 0) temp = 0;
//		PIDout =(int16_t)(((int64_t) T_MAX * temp) >> 24);												// q16*q24 = q40 >> 24 -> q16
//		
//		if (PIDout > T_MAX)
//		{
//			PIDout = T_MAX;
//		}
//		
//		if (PIDout < T_MIN)
//		{
//			PIDout = T_MIN;
//		}
			
//		temp_ARR = (PIDout >> 1) << 1;
//		MDR_TIMER3->ARR			= temp_ARR;
//		MDR_TIMER3->CCR1 		= PIDout >> 1;
		
		x[2] = x[1];
		x[1] = x[0];
		y[2] = y[1];
		y[1] = y[0];
				
		x[0] = 3000000.0f / ((float) enc_a + (float) enc_a);
		y[0] = ACoef[0] * x[0];		
		y[0] += ACoef[1] * x[1] - BCoef[1] * y[1] + ACoef[2] * x[2] - BCoef[2] * y[2];
		skor = y[0];
		
		//skor  = x[0] = 3000000.0f / ((float) enc_a + (float) enc_a);
		
		skor_err = skor_ref - skor;
		skor_err = skor_err / 300.0f;
		skor_sum_err += skor_err;

//		skor_err = skor_ref - (float) Cnt_Actuator;
//		skor_err = skor_err / 600;
//		skor_sum_err += skor_err;
		
		if (skor_sum_err > 50.0f)
		{
			skor_sum_err = 50.0f;
		} else if (skor_sum_err < -50.0f)
		{
			skor_sum_err = -50.0f;
		}
		skor_PID = Skor_kp * skor_err + Skor_ki * skor_sum_err;
		
		if (skor_PID > 0.5f)
		{
			skor_PID = 0.5f;
		}
		else if (skor_PID < -0.5f)
		{
			skor_PID = -0.5f;
		}
		
//		if ((skor_PID < -0.5) && (skor_ref < 0))
//		{
//			skor_PID = -0.5;
//		}
//		else if ((skor_PID > 0) && (skor_ref < 0))
//		{
//			skor_PID = 0.0;
//		}
		
		if (flags & (1<<FORWARD))
		{
			MDR_TIMER3->CCR1 = (uint16_t) (400 - (380 * skor_PID));
			MDR_TIMER3->CCR2 = (uint16_t) (400 + (380 * skor_PID));
		}
		else
		{
			MDR_TIMER3->CCR1 = (uint16_t) (400 + (380 * skor_PID));
			MDR_TIMER3->CCR2 = (uint16_t) (400 - (380 * skor_PID));
		}
	}
}
void Timer2_IRQHandler ()
{
	uint8_t temp;
	static uint16_t enc_a_pr=0, enc_b_pr=0;
	if (cnt_speed_0 > 0)
	{
		cnt_speed_0 -= 2;
	}
//	status = MDR_TIMER2->STATUS;
//	Cnt_interrupt++;
	
	if (MDR_TIMER2->STATUS & TIMER_STATUS_CCR_CAP_CH1)
	{	
		MDR_PORTD->RXTX ^= (1<<PD6);
		//MDR_TIMER2->CNT = 0;
		MDR_PORTE->FUNC &=~0x30;																				// переключение порта PE2 на вход
		MDR_TIMER2->STATUS &=~TIMER_STATUS_CCR_CAP_CH1;
		temp = 	(MDR_PORTE->RXTX & PORT_Pin_2) >> 2;
		temp += (MDR_PORTE->RXTX & PORT_Pin_2) >> 2;
		enc_a = MDR_TIMER2->CCR1 - enc_a_pr;
		enc_a_pr = MDR_TIMER2->CCR1;
		if (bufpr < bufpr_end)
		{
			*bufpr++ = (float) enc_a;
		}
		
		temp += (MDR_PORTE->RXTX & PORT_Pin_2) >> 2;
		temp += (MDR_PORTE->RXTX & PORT_Pin_2) >> 2;
		if (temp >> 2)
		{
			Cnt_Actuator++;
		}
		else
		{
			Cnt_Actuator--;
		}
		MDR_PORTE->FUNC |=0x20;																				// переключение порта PE2 на вход таймера
	}
	if (MDR_TIMER2->STATUS & TIMER_STATUS_CCR_CAP_CH3)
	{
		MDR_PORTE->FUNC &=~0x03;
		MDR_TIMER2->STATUS &=~TIMER_STATUS_CCR_CAP_CH3;
		temp = 	(MDR_PORTE->RXTX & PORT_Pin_0);
		temp += (MDR_PORTE->RXTX & PORT_Pin_0);
		enc_b = MDR_TIMER2->CCR3 - enc_b_pr;
		enc_b_pr = MDR_TIMER2->CCR3;
		
		if (bufpr < bufpr_end)
		{
			*bufpr++ = (float)enc_b;
		}
		
		temp += (MDR_PORTE->RXTX & PORT_Pin_0);
		temp += (MDR_PORTE->RXTX & PORT_Pin_0);
		if (temp >> 2)
		{
			Cnt_Actuator--;
		}
		else
		{
			Cnt_Actuator++;
		}
		MDR_PORTE->FUNC |=0x02;
	}
//	if (MDR_TIMER2->STATUS & TIMER_STATUS_CNT_ARR)
//	{
//		MDR_TIMER2->STATUS &=~TIMER_STATUS_CNT_ARR;
//		
//	}
//	MDR_TIMER2->STATUS = 0;
}
