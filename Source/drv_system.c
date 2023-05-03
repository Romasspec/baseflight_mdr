#include "drv_system.h"
#include "can.h"
#include "uart.h"

// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;

// SysTick
void SysTick_Handler(void)
{
	sysTickUptime++;	
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do {
		ms = sysTickUptime;
		cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void delayMicroseconds(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}

void delay(uint32_t ms)
{
    while (ms--)
		{
			delayMicroseconds(1000);
		}        
}

void failureMode(uint8_t mode)
{
//		LED1_OFF;
// 		LED0_ON;
    while (1)
		{
//     LED1_TOGGLE;
//     LED0_TOGGLE;
       delay(475 * mode - 2);

//     BEEP_ON			
       delay(25);

//     BEEP_OFF;
		}
}

void hardwareInit(void)
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
	
	PORT_InitTypeDef Port_Initstructure;
	
	PORT_DeInit (MDR_PORTA);
	PORT_DeInit (MDR_PORTB);
	PORT_DeInit (MDR_PORTC);
	PORT_DeInit (MDR_PORTD);
	PORT_DeInit (MDR_PORTE);
	PORT_DeInit (MDR_PORTF);
	
	
//	PORT_StructInit (&Port_Initstructure);
//	// Configure PORTB pins  for output
//	Port_Initstructure.PORT_Pin		= PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3;
//	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
//	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
//	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
//	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
//	PORT_Init (MDR_PORTB, &Port_Initstructure);
//	
//	// Configure PORTC pins  for output
//	PORT_StructInit (&Port_Initstructure);
//	Port_Initstructure.PORT_Pin		= PORT_Pin_1;
//	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
//	Port_Initstructure.PORT_FUNC	= PORT_FUNC_PORT;
//	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
//	Port_Initstructure.PORT_SPEED	= PORT_SPEED_SLOW;
//	PORT_Init (MDR_PORTC, &Port_Initstructure);
//	
//	// Configure PORTD pins  for output
//	PORT_StructInit (&Port_Initstructure);
//	Port_Initstructure.PORT_Pin		= PORT_Pin_6 | PORT_Pin_7;
//	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
//	Port_Initstructure.PORT_FUNC	= PORT_FUNC_PORT;
//	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
//	Port_Initstructure.PORT_SPEED	= PORT_SPEED_SLOW;
//	PORT_Init (MDR_PORTD, &Port_Initstructure);
//	

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
//	
//	// Configure PORTE pins  for input
//	PORT_StructInit (&Port_Initstructure);
//	Port_Initstructure.PORT_Pin		= PORT_Pin_0 | PORT_Pin_2;
////	Port_Initstructure.PORT_OE		= PORT_OE_IN;
//	Port_Initstructure.PORT_FUNC	= PORT_FUNC_ALTER;
//	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
//	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
//	PORT_Init (MDR_PORTE, &Port_Initstructure);


//	Port_Initstructure.PORT_Pin		= PORT_Pin_2;
//	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
//	Port_Initstructure.PORT_FUNC	= PORT_FUNC_PORT;
//	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
//	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
//	PORT_Init (MDR_PORTD, &Port_Initstructure);
	
	Port_Initstructure.PORT_Pin		= PORT_Pin_4|PORT_Pin_6;
	Port_Initstructure.PORT_OE		= PORT_OE_OUT;
	Port_Initstructure.PORT_FUNC	= PORT_FUNC_PORT;
	Port_Initstructure.PORT_MODE	= PORT_MODE_DIGITAL;
	Port_Initstructure.PORT_SPEED	= PORT_SPEED_MAXFAST;
	PORT_Init (MDR_PORTF, &Port_Initstructure);
	
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
	
	// Init cycle counter
	cycleCounterInit();
		
	// SysTick
	SysTick_Config (SystemCoreClock/1000);
	
	canInit();
	uartInit();	
}


static void cycleCounterInit(void)
{
	RST_CLK_FreqTypeDef clocks;
	RST_CLK_GetClocksFreq (&clocks);
	usTicks = clocks.CPU_CLK_Frequency / 1000000;
}

void systemReset(bool toBootloader)
{
	
	MDR_WWDG->CFR	= 0x00000000;
	MDR_WWDG->CR	= 0x000000F1;
}

