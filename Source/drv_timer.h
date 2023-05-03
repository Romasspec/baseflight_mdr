#pragma once

typedef void timerCCCallbackPtr(uint8_t port, uint16_t capture);

typedef struct {
    MDR_TIMER_TypeDef *tim;
    MDR_PORT_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    IRQn_Type irq;
		PORT_FUNC_TypeDef func;
    uint8_t outputEnable;
} timerHardware_t;

extern const timerHardware_t timerHardware[];

void configTimeBase(MDR_TIMER_TypeDef *tim, uint16_t period, uint8_t mhz);
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);
void timerNVICConfigure(IRQn_Type irq);
static uint8_t lookupTimerIndex(const MDR_TIMER_TypeDef *tim);
static uint8_t lookupChannelIndex(const int channel);
void configureTimerChannelCallback(MDR_TIMER_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback);
void configureTimerInputCaptureCompareChannel(MDR_TIMER_TypeDef *tim, const uint8_t channel);

