#include "board.h"

typedef struct {
    volatile uint16_t *ccr;
    volatile uint16_t *cr1;
    volatile uint16_t *cnt;
    uint16_t period;

    // for input only
    uint8_t channel;
    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;
} pwmPortData_t;

enum {
    TYPE_IP = 0x10,
    TYPE_IW = 0x20,
    TYPE_M = 0x40,
    TYPE_S = 0x80
};

typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors

static bool syncPWM = false;
static uint16_t failsafeThreshold = 985;
static uint8_t numInputs = 0;
static uint8_t numMotors = 0;
static uint8_t numServos = 0;
static pwmPortData_t pwmPorts[MAX_PORTS];
static pwmPortData_t *motors[MAX_MOTORS];
static pwmPortData_t *servos[MAX_SERVOS];
 uint16_t captures[MAX_PPM_INPUTS];    				// max out the captures array, just in case...
static pwmWriteFuncPtr pwmWritePtr = NULL;

static const uint8_t multiPWM[] = {						// таблица настройки таймеров, на вход в режим захвата или на выход
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,     // input #2
    PWM3 | TYPE_IW,     // input #3
    PWM4 | TYPE_IW,     // input #4
    PWM5 | TYPE_IW,     // input #5
    PWM6 | TYPE_M,      // motor #1 or servo #1 (swap to servo if needed)
    PWM7 | TYPE_M,      // motor #2 or servo #2 (swap to servo if needed)
    PWM8 | TYPE_M,      // motor #3 or servo #3 (swap to servo if needed)
    PWM9 | TYPE_M,      // motor #4 or servo #4 (swap to servo if needed)
    0xFF
};

static const uint8_t multiPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM6 | TYPE_M,      // Swap to servo if needed
    PWM7 | TYPE_M,      // Swap to servo if needed
    PWM8 | TYPE_M,      // Swap to servo if needed
    PWM9 | TYPE_M,      // Swap to servo if needed
//    PWM5 | TYPE_M,      // Swap to servo if needed
    0xFF
};

static const uint8_t airPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // motor #1
    PWM5 | TYPE_S,      // servo #5
    PWM6 | TYPE_S,
    PWM7 | TYPE_S,
    PWM8 | TYPE_S,      // servo #8
    0xFF
};

static const uint8_t airPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1
    0xFF
};

static const uint8_t *const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

#define PWM_TIMER_MHZ 1
#define PWM_TIMER_8_MHZ 8

static void pwmOCConfig(MDR_TIMER_TypeDef *tim, uint8_t channel, uint16_t value)
{
//	uint16_t tim_oc_preload;
	TIMER_ChnOutInitTypeDef TIM_OCInitStructure;
	TIMER_ChnInitTypeDef TIM_ICInitStructure;

	TIMER_ChnStructInit (&TIM_ICInitStructure);
	TIM_ICInitStructure.TIMER_CH_Number = (uint32_t) channel;
	TIM_ICInitStructure.TIMER_CH_Mode = TIMER_CH_MODE_PWM;
	TIM_ICInitStructure.TIMER_CH_REF_Format = TIMER_CH_REF_Format6;
	if (syncPWM)
	{
		TIM_ICInitStructure.TIMER_CH_CCR_UpdateMode = TIMER_CH_CCR_Update_Immediately;
	} else
	{
		TIM_ICInitStructure.TIMER_CH_CCR_UpdateMode = TIMER_CH_CCR_Update_On_CNT_eq_0;
	}	
	TIMER_ChnInit (tim, &TIM_ICInitStructure);
  
	TIMER_ChnOutStructInit (&TIM_OCInitStructure);
	TIM_OCInitStructure.TIMER_CH_Number = channel;
	TIM_OCInitStructure.TIMER_CH_DirOut_Polarity = TIMER_CHOPolarity_NonInverted;
	TIM_OCInitStructure.TIMER_CH_DirOut_Source = TIMER_CH_OutSrc_REF;
	TIM_OCInitStructure.TIMER_CH_DirOut_Mode = TIMER_CH_OutMode_Output;
	TIMER_ChnOutInit (tim, &TIM_OCInitStructure);
	
	TIMER_SetChnCompare (tim, channel, value);
}

static pwmPortData_t *pwmOutConfig(uint8_t port, uint8_t mhz, uint16_t period, uint16_t value)
{
	pwmPortData_t *p = &pwmPorts[port];
	configTimeBase (timerHardware[port].tim, period, mhz);
	pwmGPIOConfig (timerHardware[port].gpio, timerHardware[port].pin, timerHardware[port].func);
	pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, value);
	TIMER_Cmd(timerHardware[port].tim, ENABLE);
	
	p->cr1 = (uint16_t *)(&timerHardware[port].tim->CNTRL);
	p->ccr = (uint16_t *)(&timerHardware[port].tim->CNT);
	
	switch (timerHardware[port].channel)
	{
		case TIMER_CHANNEL1:
			p->ccr = (uint16_t *) (&timerHardware[port].tim->CCR1);
			break;
		case TIMER_CHANNEL2:
			p->ccr = (uint16_t *) (&timerHardware[port].tim->CCR2);
			break;
		case TIMER_CHANNEL3:
			p->ccr = (uint16_t *) (&timerHardware[port].tim->CCR3);
			break;
		case TIMER_CHANNEL4:
			p->ccr = (uint16_t *) (&timerHardware[port].tim->CCR4);
			break;		
	}
	
	p->period = period;
	return p;
}

static pwmPortData_t *pwmInConfig(uint8_t port, timerCCCallbackPtr callback, uint8_t channel)
{
	pwmPortData_t *p = &pwmPorts[port];
	const timerHardware_t *timerHardwarePtr = &(timerHardware[port]);

	p->channel = channel;

	pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->func);
	pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIMER_CH_EvSrc_PE);
	timerConfigure(timerHardwarePtr, 0xFFFF, PWM_TIMER_MHZ);
	configureTimerCaptureCompareInterrupt(timerHardwarePtr, port, callback);
	
	return p;
}

void pwmICConfig(MDR_TIMER_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
	TIMER_ChnInitTypeDef TIM_ICInitStructure;

	TIMER_ChnStructInit (&TIM_ICInitStructure);
	TIM_ICInitStructure.TIMER_CH_Number = (uint32_t) channel;
	TIM_ICInitStructure.TIMER_CH_Mode = TIMER_CH_MODE_CAPTURE;
	TIM_ICInitStructure.TIMER_CH_Prescaler = TIMER_CH_Prescaler_None;
	TIM_ICInitStructure.TIMER_CH_EventSource = (uint32_t) polarity;
	TIM_ICInitStructure.TIMER_CH_FilterConf = TIMER_Filter_1FF_at_TIMER_CLK;
	TIM_ICInitStructure.TIMER_CH_CCR_UpdateMode = TIMER_CH_CCR_Update_Immediately;
	TIM_ICInitStructure.TIMER_CH_CCR1_Ena = DISABLE;
	TIM_ICInitStructure.TIMER_CH_CCR1_EventSource = TIMER_CH_CCR1EvSrc_NE;

	TIMER_ChnInit (tim, &TIM_ICInitStructure);
}
static void pwmGPIOConfig(MDR_PORT_TypeDef *gpio, uint32_t pin, PORT_FUNC_TypeDef func)
{
	PORT_InitTypeDef cfg;
	
	cfg.PORT_Pin = pin;
	cfg.PORT_SPEED = PORT_SPEED_MAXFAST;
	cfg.PORT_FUNC = func;
	cfg.PORT_MODE = PORT_MODE_DIGITAL;
	cfg.PORT_OE = PORT_OE_IN;
	cfg.PORT_PULL_UP = PORT_PULL_UP_OFF;
	cfg.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
	cfg.PORT_PD = PORT_PD_DRIVER;
	cfg.PORT_PD_SHM = PORT_PD_SHM_ON;
	cfg.PORT_GFEN = PORT_GFEN_ON;	
	PORT_Init (gpio, &cfg);
}

void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback)
{
	configureTimerChannelCallback(timerHardwarePtr->tim, timerHardwarePtr->channel, reference, callback);
	configureTimerInputCaptureCompareChannel(timerHardwarePtr->tim, timerHardwarePtr->channel);
}

void timerNVICConfigure(IRQn_Type irq)
{
	NVIC_SetPriority (irq, 2);
	NVIC_EnableIRQ(irq);
}

void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
	configTimeBase(timerHardwarePtr->tim, period, mhz);
	TIMER_Cmd (timerHardwarePtr->tim, ENABLE);
	timerNVICConfigure(timerHardwarePtr->irq);
}

static void ppmCallback(uint8_t port, uint16_t capture)
{
	uint16_t diff;
	static uint16_t now;
	static uint16_t last = 0;
	static uint8_t chan = 0;
	togle_PF6;
	last = now;
	now = capture;
	diff = now - last;


	if (diff > 2700/* || chan > MAX_PPM_INPUTS*/) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
			chan = 0;
	} else {
			if (diff > PULSE_MIN && diff < PULSE_MAX && chan < MAX_PPM_INPUTS) {   // 750 to 2250 ms is our 'valid' channel range
					captures[chan] = diff;
//					failsafeCheck(chan, diff);
			}
			chan++;
	}
}

static void pwmCallback(uint8_t port, uint16_t capture)								//вызывается при каждом прерывании таймера по захвату
{
	if(pwmPorts[port].state == 0)
	{
		pwmPorts[port].rise = capture;
		pwmPorts[port].state = 1;
		pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIMER_CH_EvSrc_NE);
	} else
	{
		pwmPorts[port].fall = capture;
		pwmPorts[port].capture = pwmPorts[port].fall - pwmPorts[port].rise;
		
		if(pwmPorts[port].capture > PULSE_MIN && pwmPorts[port].capture < PULSE_MAX)							// valid pulse width
		{
			captures[pwmPorts[port].channel] = pwmPorts[port].capture;
//			failsafeCheck(pwmPorts[port].channel, pwmPorts[port].capture);
		}		
		// switch state
		pwmPorts[port].state = 0;
		pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIMER_CH_EvSrc_PE);
	}
}

static void pwmWriteBrushed(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = (value - 1000) * motors[index]->period / 1000;
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = value;
}

static void pwmWriteSyncPwm(uint8_t index, uint16_t value)
{
    *motors[index]->cr1 &= (uint16_t) ~(0x0001);    // disable timer
    *motors[index]->cnt = 0x0000;                   // set timer counter to zero
    *motors[index]->ccr = value;                    // set the pwm value
    *motors[index]->cr1 |= (uint16_t) (0x0001);     // enable timer
}

bool pwmInit(drv_pwm_config_t *init)
{
	int i = 0;
	uint16_t period;
	const uint8_t *setup;
	
	// to avoid importing cfg/mcfg
    failsafeThreshold = init->failsafeThreshold;
	
	// this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM)
        i++; // next index is for PPM
	
	setup = hardwareMaps[i];
	
	for(i = 0; i < MAX_PORTS; i++)
	{
		uint8_t port = setup[i] & 0x0F;
		uint8_t mask = setup[i] & 0xF0;
		if (setup[i] == 0xFF) break;								// terminator            
		
		if (mask & TYPE_IP)
		{			
			pwmInConfig(port, ppmCallback, 0);
			numInputs = 8;
		} else if (mask & TYPE_IW)
		{
			pwmInConfig(port, pwmCallback, numInputs);
			numInputs++;
		} else if (mask & TYPE_M)
		{
			uint32_t hz, mhz;

			if (init->motorPwmRate > 500 || init->fastPWM)
			{
				mhz = PWM_TIMER_8_MHZ;
			} else
			{
				mhz = PWM_TIMER_MHZ;
			}
			
			hz = mhz * 1000000;
			
			if (init->syncPWM)
			{
				period = 8000 * mhz; // 8ms period in syncPWM mode, cycletime should be smaller than this
			} else if (init->fastPWM)
			{
				period = hz / 4000;
			} else
			{
				period = hz / init->motorPwmRate;
			}
			
			motors[numMotors++] = pwmOutConfig(port, mhz, period, init->idlePulse);
		}
	}
	
	// determine motor writer function
	pwmWritePtr = pwmWriteStandard;
	if (init->motorPwmRate > 500)
	{
		pwmWritePtr = pwmWriteBrushed;
	} else if (init->syncPWM)
	{
		pwmWritePtr = pwmWriteSyncPwm;
	}
	// set return values in init struct
	init->numServos = numServos;
	
	return false;
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
	if (index < numMotors)
	{
		pwmWritePtr(index, value);
	}        
}

void pwmWriteServo(uint8_t index, uint16_t value)
{
	if (index < numServos)
	{
		*servos[index]->ccr = value;
	}
}

