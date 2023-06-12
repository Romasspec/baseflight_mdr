#include "board.h"
#include "uart.h"

#define I2C_DEFAULT_TIMEOUT 300000
//#define DEBUG_I2C
static volatile uint16_t i2cErrorCount = 0;

static volatile bool error = false;
static volatile bool busy;

static volatile uint8_t addr;
static volatile uint8_t reg;
static volatile uint8_t bytes;
static volatile uint8_t writing;
static volatile uint8_t reading;
static volatile uint8_t *write_p;
static volatile uint8_t *read_p;

static volatile uint8_t flag_I2c;

#ifdef DEBUG_I2C
	#define SIZE_DEBUG 	15
	uint8_t data_debug[SIZE_DEBUG], debug_index = 0;
#endif

#define START_SENT 		(uint8_t) (0x01)
#define ADDRES_SENT		(uint8_t) (0x02)
#define READ_BYTE_1		(uint8_t) (0x04)
#define STOP_SENT 		(uint8_t) (0x08)
#define ERROR_SENT 		(uint8_t) (0x10)
//#define READ					(uint8_t) (0x08)

typedef struct i2cDevice_t {
	MDR_I2C_TypeDef *dev;
	MDR_PORT_TypeDef *gpio;
	uint16_t scl;
	uint16_t sda;
	IRQn_Type i2c_irq;
	uint32_t peripheral;
} i2cDevice_t;

static const i2cDevice_t i2cHardwareMap[] = {
	{MDR_I2C, MDR_PORTC, PORT_Pin_0, PORT_Pin_1, I2C_IRQn, RST_CLK_PCLK_I2C},
	{MDR_I2C, MDR_PORTC, PORT_Pin_0, PORT_Pin_1, I2C_IRQn, RST_CLK_PCLK_I2C}
};
// Copy of peripheral address for IRQ routines
static MDR_I2C_TypeDef *I2Cx = NULL;
// Copy of device index for reinit, etc purposes
static I2CDevice I2Cx_index;

static bool i2cHandleHardwareFailure(void)
{
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    i2cInit(I2Cx_index);
		
    return false;
}

static void i2cUnstick(void)
{	
	MDR_PORT_TypeDef *gpio;
	PORT_InitTypeDef cfg;
	uint16_t scl, sda;
	uint8_t i;
	
	// prepare pins
	gpio = i2cHardwareMap[I2Cx_index].gpio;
	scl = i2cHardwareMap[I2Cx_index].scl;
	sda = i2cHardwareMap[I2Cx_index].sda;
	
	digitalHi (gpio, scl|sda);
	
	cfg.PORT_Pin = scl|sda;
	cfg.PORT_SPEED = PORT_SPEED_MAXFAST;
	cfg.PORT_FUNC = PORT_FUNC_PORT;
	cfg.PORT_MODE = PORT_MODE_DIGITAL;
	cfg.PORT_OE = PORT_OE_OUT;
	cfg.PORT_PULL_UP = PORT_PULL_UP_ON;
	cfg.PORT_PD = PORT_PD_OPEN;
	
	PORT_Init (gpio, &cfg);
	
	for (i = 0; i < 8; i++)
	{
		// Wait for any clock stretching to finish
		while (!digitalIn(gpio, scl))
		{
			delayMicroseconds(10);
		}
		// Pull low
		digitalLo(gpio, scl); // Set bus low
		delayMicroseconds(10);
		// Release high again
		digitalHi(gpio, scl); // Set bus high
		delayMicroseconds(10);
   }

		// Generate a start then stop condition
    // SCL  PB10
    // SDA  PB11
    digitalLo(gpio, sda); // Set bus data low
    delayMicroseconds(10);
    digitalLo(gpio, scl); // Set bus scl low
    delayMicroseconds(10);
    digitalHi(gpio, scl); // Set bus scl high
    delayMicroseconds(10);
    digitalHi(gpio, sda); // Set bus sda high

    // Init pins
    cfg.PORT_Pin = scl|sda;
		cfg.PORT_SPEED = PORT_SPEED_MAXFAST;
		cfg.PORT_FUNC = PORT_FUNC_ALTER;
		cfg.PORT_MODE = PORT_MODE_DIGITAL;
		cfg.PORT_OE = PORT_OE_OUT;
		cfg.PORT_PULL_UP = PORT_PULL_UP_ON;
		cfg.PORT_PD = PORT_PD_OPEN;
    PORT_Init(gpio, &cfg);
	}

void i2cInit(I2CDevice index)
{
	I2C_InitTypeDef igc;
	I2Cx_index = index;
	
	if (index > I2CDEV_MAX)
	{
		index = I2CDEV_MAX;		
	}
	
	// Turn on peripheral clock, save device and index
	I2Cx = i2cHardwareMap[index].dev;
	RST_CLK_PCLKcmd ( i2cHardwareMap[index].peripheral, ENABLE);
	
	// clock out stuff to make sure slaves arent stuck
	// This will also configure GPIO as AF_OD at the end
	i2cUnstick();	
	
	I2C_DeInit();
	
	I2C_ITConfig (DISABLE);				// Enable EVT and ERR interrupts - they are enabled by the first request
	I2C_Cmd (ENABLE);
	
	I2C_StructInit (&igc);
	igc.I2C_ClkDiv = 150;
	igc.I2C_Speed = I2C_SPEED_UP_TO_400KHz;
	I2C_Init (&igc);	
	
	// I2C Interrupt	
	NVIC_SetPriorityGrouping (5);
	NVIC_SetPriority (i2cHardwareMap[index].i2c_irq, 0);
	NVIC_EnableIRQ(i2cHardwareMap[index].i2c_irq);	
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
	//uint32_t timeout = I2C_DEFAULT_TIMEOUT;
	addr = addr_ << 1;
	reg = reg_;
	writing = 1;
	reading = 0;
	write_p = data;
	read_p = data;
	bytes = len_;
	busy = 1;
	error = false;
	
	return i2cStartTranzaction();
//	
//	if (!I2Cx) return false;
//	
//	if (!(I2Cx->CTR & I2C_CTR_EN_INT))														// проверяем отключенные прерывания
//	{
//		if(!(I2Cx->CMD & I2C_CMD_START))														// проверяем что нет команды START
//		{
//			while (I2Cx->STA & I2C_STA_TR_PROG && --timeout > 0)			// ожидаем либо таймаута либо I2C_STA_TR_PROG = 0 // I2Cx->CMD & I2C_CMD_STOP
//			{}
//			
//			if (timeout == 0) return i2cHandleHardwareFailure();
//			
//			I2C_Send7bitAddress(addr, I2C_Direction_Transmitter);
//			flag_I2c = START_SENT;
//			I2Cx->CMD |= I2C_CMD_CLRINT;
//		}
//		I2C_ITConfig (ENABLE);
//	}
//	
//	timeout = I2C_DEFAULT_TIMEOUT;
//	while (busy && --timeout > 0)	{}
//		
//	if (timeout == 0) return i2cHandleHardwareFailure();
//	
//	return !error;
}

bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(addr_, reg_, 1, &data);
}

bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
	//uint32_t timeout = I2C_DEFAULT_TIMEOUT;
	addr = addr_<<1;
	reg = reg_;
	writing = 0;
	reading = 1;
	read_p = buf;
	write_p = buf;
	bytes = len;
	busy = 1;
	error = false;
	
	return i2cStartTranzaction();
//	
//	if (!I2Cx)
//	{
//		return false;
//	}
//		
//	if (!(I2Cx->CTR & I2C_CTR_EN_INT))														// проверяем отключенные прерывания
//	{
//		if(!(I2Cx->CMD & I2C_CMD_START))														// проверяем что нет команды START
//		{
//			while (I2Cx->STA & I2C_STA_TR_PROG && --timeout > 0)			// ожидаем либо таймаута либо I2C_STA_TR_PROG = 0 // I2Cx->CMD & I2C_CMD_STOP
//			{}
//			
//			if (timeout == 0)
//			{      
//				return i2cHandleHardwareFailure();
//			}
//			I2C_Send7bitAddress(addr, I2C_Direction_Transmitter);
//			flag_I2c = START_SENT;
//			I2Cx->CMD |= I2C_CMD_CLRINT;
//		}
//		I2C_ITConfig (ENABLE);
//	}
//	
//	timeout = I2C_DEFAULT_TIMEOUT;
//	while (busy && --timeout > 0)
//	{}
//		
//	if (timeout == 0)
//	{
//		return i2cHandleHardwareFailure();
//	}
//	
//	return !error;
}
bool i2cStartTranzaction(void)
{
	uint32_t timeout = I2C_DEFAULT_TIMEOUT;
	
#ifdef DEBUG_I2C	
	for (debug_index = 0; debug_index < SIZE_DEBUG; debug_index++) {
		data_debug[debug_index] = 0;
	}
	debug_index = 0;
#endif
	
	if (!I2Cx)
	{
		return false;
	}
		
	if (!(I2Cx->CTR & I2C_CTR_EN_INT))														// проверяем отключенные прерывания
	{
		if(!(I2Cx->CMD & I2C_CMD_START))														// проверяем что нет команды START
		{
			while (I2Cx->STA & I2C_STA_TR_PROG && --timeout > 0)			// ожидаем либо таймаута либо I2C_STA_TR_PROG = 0 // I2Cx->CMD & I2C_CMD_STOP
			{}
			
			if (timeout == 0)
			{      
				return i2cHandleHardwareFailure();
			}
			I2C_Send7bitAddress(addr, I2C_Direction_Transmitter);
			flag_I2c = START_SENT;
			I2Cx->CMD |= I2C_CMD_CLRINT;
		}
		I2C_ITConfig (ENABLE);
	}
	
	timeout = I2C_DEFAULT_TIMEOUT;
	while (busy && --timeout > 0)
	{}
	
#ifdef DEBUG_I2C	
	uartTransmit(data_debug, SIZE_DEBUG);
#endif
		
	if (timeout == 0)
	{
		return i2cHandleHardwareFailure();
	}
	
	return !error;
}

void I2C_IRQHandler (void)
{
	static int8_t index;
	uint8_t SReg_1 = I2Cx->STA;                                         // read the status register here
	I2Cx->CMD |= I2C_CMD_CLRINT;

	if(!(SReg_1 & I2C_STA_RX_ACK))
	{
		if(flag_I2c & START_SENT)
		{
			index = 0;
			I2C_SendByte (reg);
			flag_I2c = ADDRES_SENT;
			I2Cx->CMD &= ~I2C_CMD_START;
#ifdef DEBUG_I2C
			data_debug[debug_index++] = 0x01;
#endif
		}
		else if (flag_I2c & ADDRES_SENT)
		{
			flag_I2c &=~ADDRES_SENT;
			if(reading)
			{
				I2C_Send7bitAddress(addr, I2C_Direction_Receiver);
				flag_I2c = READ_BYTE_1;
#ifdef DEBUG_I2C				
				data_debug[debug_index++] = 0x02;
#endif
			}
			else
			{
				I2C_SendByte(write_p[index]);
#ifdef DEBUG_I2C
				data_debug[debug_index++] = 0x03;
#endif
			}
		}
		else if (flag_I2c & READ_BYTE_1)
		{
			if(--bytes != 0)
			{				
				I2C_StartReceiveData(I2C_Send_to_Slave_ACK);
#ifdef DEBUG_I2C
				data_debug[debug_index++] = 0x04;
#endif
			}
			else
			{
				I2C_StartReceiveData(I2C_Send_to_Slave_NACK);				
#ifdef DEBUG_I2C
				data_debug[debug_index++] = 0x05;				
#endif
			}					
			flag_I2c = 0;
		}
		else if (flag_I2c & STOP_SENT)
		{
			flag_I2c = 0;
			busy = 0;
			I2C_ITConfig (DISABLE);
			I2Cx->CMD &= ~I2C_CMD_STOP;
#ifdef DEBUG_I2C			
			data_debug[debug_index++] = 0x06;
#endif
		}		
		else
		{			
			if(reading)
			{
				if(--bytes != 0)
				{	
					read_p[index++] = I2C_GetReceivedData();
					I2C_StartReceiveData(I2C_Send_to_Slave_ACK);
#ifdef DEBUG_I2C
					data_debug[debug_index++] = 0x07;
#endif
				}
				else
				{
					read_p[index++] = I2C_GetReceivedData();
					I2C_StartReceiveData(I2C_Send_to_Slave_NACK);				
#ifdef DEBUG_I2C
					data_debug[debug_index++] = 0x08;					
#endif
				}	
			}
			else
			{				
				//I2C_ITConfig (DISABLE);
				I2C_SendSTOP();
				flag_I2c = STOP_SENT;
				//busy = 0;	
#ifdef DEBUG_I2C
				data_debug[debug_index++] = 0x09;				
#endif
			}
		}		
	}
	else
	{
		
		if(flag_I2c & START_SENT)
		{
			I2C_SendSTOP();
			flag_I2c = ERROR_SENT;			
#ifdef DEBUG_I2C
			data_debug[debug_index++] = 0x0A;
#endif
		}
		else if (flag_I2c & STOP_SENT)
		{
			flag_I2c = 0;
			busy = 0;
			I2C_ITConfig (DISABLE);
			I2Cx->CMD &= ~I2C_CMD_STOP;
#ifdef DEBUG_I2C
			data_debug[debug_index++] = 0x0B;
#endif
		}
		else if (flag_I2c & ERROR_SENT)
		{
			flag_I2c = 0;			
			I2C_ITConfig (DISABLE);
			I2Cx->CMD &= ~I2C_CMD_STOP;
#ifdef DEBUG_I2C
			data_debug[debug_index++] = 0x0C;
#endif
		}
		else
		{			
			if(bytes != 0)
			{				
				read_p[index++] = I2C_GetReceivedData();
				if(--bytes != 0)
				{				
					I2C_StartReceiveData(I2C_Send_to_Slave_ACK);
#ifdef DEBUG_I2C
					data_debug[debug_index++] = 0x0D;
#endif
				}
				else
				{
					I2C_StartReceiveData(I2C_Send_to_Slave_NACK);					
#ifdef DEBUG_I2C
					data_debug[debug_index++] = 0x0E;					
#endif
				}		
			}
			else
			{				
				read_p[index] = I2C_GetReceivedData();
				I2C_SendSTOP();
				flag_I2c = STOP_SENT;
#ifdef DEBUG_I2C
				data_debug[debug_index++] = 0x0F;
#endif				
			}
		}
	}
}
