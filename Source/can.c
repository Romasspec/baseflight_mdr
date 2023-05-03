#include "can.h"
#include "mw.h"

j1939msg_t CANmsg;
CAN_RxMsgTypeDef RxMsg;
CAN_TxMsgTypeDef TxMsg;
//extern master_t mcfg;
uint32_t buf[sizeof(master_t)/4];

extern uint16_t captures[MAX_PPM_INPUTS];

void canInit (void)
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

void RXcan (uint32_t *data)
{
	static uint32_t prevtime = 0;
	
	if ((MDR_CAN2->BUF_CON[1] & CAN_STATUS_RX_FULL) ==  CAN_STATUS_RX_FULL)
	{
		MDR_CAN2->BUF_CON[1] &=~CAN_STATUS_RX_FULL;
						
		CAN_GetRawReceivedData (MDR_CAN2, 1, &RxMsg);
		CANmsg.idt = RxMsg.Rx_Header.ID;
		CANmsg.data_u32[1] = RxMsg.Data[1];
		CANmsg.data_u32[0] = RxMsg.Data[0];
		
		if (CANmsg.pf == PGN_DEV_RESET)
		{
			MDR_WWDG->CFR	= 0x00000180;
			MDR_WWDG->CR	 = 0x7F;
			MDR_WWDG->CR	|= 0x80;					
		}
		
		if (CANmsg.pf == PGN_ErasePage_FLASH)
		{
			FLASH_ErasePage (CANmsg.data_u32[0], EEPROM_Info_Bank_Select);			
		}
		
		if (CANmsg.pf == PGN_READ_FLASH)
		{
			memset(buf, 0xAA, sizeof (buf));	
//			FLASH_ReadWord (CANmsg.data_u32[0], buf, EEPROM_Info_Bank_Select);
//			uartTransmit ((uint8_t*) buf, 4);

			
//			UART_SendData (MDR_UART2, 0x11);
			for (uint16_t i=0; i < sizeof(master_t); i+=4)
			{				
				FLASH_ReadWord (CANmsg.data_u32[0]+i, (uint32_t*)((uint8_t*)buf + i), EEPROM_Info_Bank_Select);				
			}
			uartTransmit ((uint8_t*) buf, sizeof(master_t));				
		}
		
		if (CANmsg.pf == PGN_WRITE_FLASH)
		{
			FLASH_ProgramWord (CANmsg.data_u32[0], CANmsg.data_u32[1], EEPROM_Info_Bank_Select);
		}
		
		if (CANmsg.pf == PGN_i2cRead)
		{
			if (i2cRead (0x68, CANmsg.data[0], CANmsg.data[1], (uint8_t *)buf) != false)
			{
				uartTransmit ((uint8_t *)buf, CANmsg.data[1]);
			}			
		}
		
		if (CANmsg.pf == PGN_i2cWrite)
		{			
//			i2cWrite(0x68, 0x6B, 0x80);
//			delay(100);
			*((uint8_t *) buf) = CANmsg.data[1];
			if(i2cWrite(0x68, CANmsg.data[0], *((uint8_t*) buf)) != false)
			{
				*((uint8_t *) buf) = 0xFF;
			}
			else
			{
				*((uint8_t *) buf) = 0x00;
			}
			uartTransmit ((uint8_t *)buf, 1);
		}
		
		if (CANmsg.pf == PGN_readReg)
		{
			//uint32_t *reg = (uint32_t*)(CANmsg.data_u32[0]);
			TxMsg.Data[0] = *((uint32_t*)(CANmsg.data_u32[0]));
			TxMsg.Data[1] = *((uint32_t*)(CANmsg.data_u32[0]+4));
			CANmsg.p		= 7;
			CANmsg.r		= 0;
			CANmsg.dp		= 0;
			CANmsg.pf 	= 0x02;
			CANmsg.ps	 	= 0x00;
			CANmsg.sa 	= 0x07;
		
			CANmsg.len	=	8;
			
			TxMsg.ID 		= CANmsg.idt;
			TxMsg.DLC		= CANmsg.len;
			TxMsg.IDE		= CAN_ID_EXT;			
			
			CAN_Transmit (MDR_CAN2, 0, &TxMsg);
		}
	}
	
	
//	static uint32_t len = 4;
	if (micros() - prevtime > 1000000) //&& (len-- != 0))
	{
		prevtime =	micros();
		static uint32_t offset = 0;
		 
		CANmsg.p		= 7;
		CANmsg.r		= 0;
		CANmsg.dp		= 0;
		CANmsg.pf 	= 0x01;
		CANmsg.ps	 	= 0x00;
		CANmsg.sa 	= 0x07;
		
		CANmsg.len	=	8;
		
//				CANmsg.data[0]	= ADC_Result & 0xFF;
//				CANmsg.data[1]	= (ADC_Result >> 8) & 0xFF;
		
		TxMsg.ID 		= CANmsg.idt;
		TxMsg.DLC		= CANmsg.len;
		TxMsg.IDE		= CAN_ID_EXT;
//		TxMsg.Data[0] = *(data + offset);
//		TxMsg.Data[1] = *(data + offset+4);
//		offset +=8;
		TxMsg.Data[0] = *captures;
		TxMsg.Data[1] = *(captures+4);
		
		CAN_Transmit (MDR_CAN2, 0, &TxMsg);
	}	
}
