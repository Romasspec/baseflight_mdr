#include "board.h"

const timerHardware_t timerHardware[] = {
    { MDR_TIMER1, MDR_PORTA, PORT_Pin_1, TIMER_CHANNEL1, Timer1_IRQn, PORT_FUNC_ALTER, 0, },          // PWM1
    { MDR_TIMER1, MDR_PORTA, PORT_Pin_3, TIMER_CHANNEL2, Timer1_IRQn, PORT_FUNC_ALTER, 0, },          // PWM2
    { MDR_TIMER1, MDR_PORTA, PORT_Pin_5, TIMER_CHANNEL3, Timer1_IRQn, PORT_FUNC_ALTER, 0, },          // PWM3
    { MDR_TIMER2, MDR_PORTE, PORT_Pin_0, TIMER_CHANNEL1, Timer2_IRQn, PORT_FUNC_ALTER, 0, },          // PWM4
    { MDR_TIMER2, MDR_PORTE, PORT_Pin_2, TIMER_CHANNEL3, Timer2_IRQn, PORT_FUNC_ALTER, 0, },          // PWM5
    { MDR_TIMER3, MDR_PORTB, PORT_Pin_0, TIMER_CHANNEL1, Timer3_IRQn, PORT_FUNC_ALTER, 1, },          // PWM6
    { MDR_TIMER3, MDR_PORTB, PORT_Pin_2, TIMER_CHANNEL2, Timer3_IRQn, PORT_FUNC_ALTER, 1, },          // PWM7
    { MDR_TIMER3, MDR_PORTB, PORT_Pin_5, TIMER_CHANNEL3, Timer3_IRQn, PORT_FUNC_OVERRID, 1, },          // PWM8
    { MDR_TIMER3, MDR_PORTB, PORT_Pin_7, TIMER_CHANNEL4, Timer3_IRQn, PORT_FUNC_OVERRID, 1, },    		   // PWM9
};

enum {
    TIM1_IDX = 0,
    TIM2_IDX,
    TIM3_IDX,
    MAX_TIMERS
};

#define CC_CHANNELS_PER_TIMER 4 // TIM_Channel_1..4

static const MDR_TIMER_TypeDef *const timers[MAX_TIMERS] = {MDR_TIMER1, MDR_TIMER2, MDR_TIMER3};

typedef struct channelConfig_s {
    uint16_t channel;
    uint16_t interruptBit;
    uint16_t (*TIM_GetCaptureFn)(MDR_TIMER_TypeDef *TIMx, uint32_t Channel);
} channelConfig_t;

static const channelConfig_t channels[CC_CHANNELS_PER_TIMER] = {
    { TIMER_CHANNEL1, TIMER_STATUS_CCR_CAP_CH1, TIMER_GetChnCapture },
    { TIMER_CHANNEL2, TIMER_STATUS_CCR_CAP_CH2, TIMER_GetChnCapture },
    { TIMER_CHANNEL3, TIMER_STATUS_CCR_CAP_CH3, TIMER_GetChnCapture },
    { TIMER_CHANNEL4, TIMER_STATUS_CCR_CAP_CH4, TIMER_GetChnCapture }
};

typedef struct timerConfig_s {
    MDR_TIMER_TypeDef *tim;
    uint8_t channel;
    timerCCCallbackPtr *callback;
    uint8_t reference;
} timerConfig_t;

timerConfig_t timerConfigs[MAX_TIMERS][CC_CHANNELS_PER_TIMER];

void configTimeBase(MDR_TIMER_TypeDef *tim, uint16_t period, uint8_t mhz)
{
	TIMER_CntInitTypeDef TIM_TimeBaseStructure;
	
	TIMER_CntStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIMER_IniCounter = 0;
	TIM_TimeBaseStructure.TIMER_Period = period;
	// "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
	// Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
	TIM_TimeBaseStructure.TIMER_Prescaler = (SystemCoreClock / ((uint32_t) mhz * 1000000))-1;
	TIM_TimeBaseStructure.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
	TIM_TimeBaseStructure.TIMER_CounterDirection = TIMER_CntDir_Up;
	TIM_TimeBaseStructure.TIMER_EventSource = TIMER_EvSrc_None;
	TIM_TimeBaseStructure.TIMER_FilterSampling = TIMER_FDTS_TIMER_CLK_div_1;
	TIM_TimeBaseStructure.TIMER_ARR_UpdateMode = TIMER_ARR_Update_Immediately;
	
	TIMER_CntInit(tim, &TIM_TimeBaseStructure);
	
	TIMER_BRGInit(tim, TIMER_HCLKdiv1);
}

void configureTimerChannelCallback(MDR_TIMER_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback)
{
	uint8_t timerIndex = lookupTimerIndex(tim);
	uint8_t channelIndex = lookupChannelIndex(channel);
	
	if (timerIndex >= MAX_TIMERS || channelIndex >= CC_CHANNELS_PER_TIMER)
	{
		return;
  }
	timerConfigs[timerIndex][channelIndex].callback = callback;
	timerConfigs[timerIndex][channelIndex].channel = channel;
	timerConfigs[timerIndex][channelIndex].reference = reference;
}

static uint8_t lookupTimerIndex(const MDR_TIMER_TypeDef *tim)
{
	uint8_t timerIndex;
	for (timerIndex = 0; timerIndex < MAX_TIMERS; timerIndex++ )
	{
		if (timers[timerIndex] == tim)
			break;
	}
	return timerIndex;
}

static uint8_t lookupChannelIndex(const int channel)
{
	uint8_t channelIndex;
	for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++ )
	{
		if (channels[channelIndex].channel == channel)
			break;
	}
	return channelIndex;
}

void configureTimerInputCaptureCompareChannel(MDR_TIMER_TypeDef *tim, const uint8_t channel)
{
	switch (channel) {
			case TIMER_CHANNEL1:
					TIMER_ITConfig(tim, TIMER_STATUS_CCR_CAP_CH1, ENABLE);
					break;
			case TIMER_CHANNEL2:
					TIMER_ITConfig(tim, TIMER_STATUS_CCR_CAP_CH2, ENABLE);
					break;
			case TIMER_CHANNEL3:
					TIMER_ITConfig(tim, TIMER_STATUS_CCR_CAP_CH3, ENABLE);
					break;
			case TIMER_CHANNEL4:
					TIMER_ITConfig(tim, TIMER_STATUS_CCR_CAP_CH4, ENABLE);
					break;
    }
}

static timerConfig_t *findTimerConfig(unsigned int timerIndex, unsigned int channelIndex)
{
	return &(timerConfigs[timerIndex][channelIndex]);
}

static void timCCxHandler(MDR_TIMER_TypeDef *const tim, uint8_t timerIndex)
{
	uint8_t channelIndex;
	
	for(channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++)
	{
		const channelConfig_t *channel = &channels[channelIndex];
		
		if(TIMER_GetITStatus(tim, channel->interruptBit) == SET)
		{
			TIMER_ClearITPendingBit(tim, channel->interruptBit);
			
			uint16_t capture;
			capture = channel->TIM_GetCaptureFn(tim, channel->channel);
			
			timerConfig_t *timerConfig;
			timerConfig = findTimerConfig(timerIndex, channelIndex);
			
			if(timerConfig->callback)
			{				
				timerConfig->callback(timerConfig->reference, capture);
			}
		}
	}
}

void Timer1_IRQHandler (void)
{
	timCCxHandler(MDR_TIMER1, TIM1_IDX);
}

void Timer2_IRQHandler (void)
{
	timCCxHandler(MDR_TIMER2, TIM2_IDX);
}

void Timer3_IRQHandler (void)
{
	timCCxHandler(MDR_TIMER3, TIM3_IDX);
}
