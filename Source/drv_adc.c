#include "board.h"

typedef struct adc_config_t {
    uint8_t adcChannel;         // ADC1_INxx channel number
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
} adc_config_t;

static adc_config_t adcConfig[ADC_CHANNEL_MAX];
static volatile uint16_t adcValues[ADC_CHANNEL_MAX];

void adcInit(drv_adc_config_t *init)
{	
	ADC_InitTypeDef adc;
	ADCx_InitTypeDef adcx;
	DMA_CtrlDataInitTypeDef dma;
	DMA_ChannelInitTypeDef dma_ch;
	int numChannels = 1;

 // configure always-present battery index (ADC2)
	adcConfig[ADC_BATTERY].adcChannel = ADC_CH_ADC2;
	adcConfig[ADC_BATTERY].dmaIndex = numChannels - 1;

 // optional ADC5 input on rev.5 hardware
//	if (1) {
//			numChannels++;
//			adcConfig[ADC_EXTERNAL_PAD].adcChannel = ADC_CH_ADC5;
//			adcConfig[ADC_EXTERNAL_PAD].dmaIndex = numChannels - 1;
//	}
	// another channel can be stolen from PWM for current measurement or other things
	if (init->powerAdcChannel > 0) {
			numChannels++;
			adcConfig[ADC_EXTERNAL_CURRENT].adcChannel = init->powerAdcChannel;
			adcConfig[ADC_EXTERNAL_CURRENT].dmaIndex = numChannels - 1;
	}

	if (init->rssiAdcChannel > 0) {
			numChannels++;
			adcConfig[ADC_RSSI].adcChannel = init->rssiAdcChannel;
			adcConfig[ADC_RSSI].dmaIndex = numChannels - 1;
	}
	
	DMA_DeInit ();	
	dma.DMA_SourceBaseAddr = (uint32_t)&MDR_ADC->ADC1_RESULT;
	dma.DMA_DestBaseAddr = (uint32_t) adcValues;
	dma.DMA_SourceIncSize = DMA_SourceIncNo;
	dma.DMA_DestIncSize = numChannels > 1 ? DMA_DestIncHalfword : DMA_DestIncNo;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_Mode = DMA_Mode_Basic;
	dma.DMA_CycleSize = ADC_CHANNEL_MAX;
	dma.DMA_NumContinuous = DMA_Transfers_1;
	dma.DMA_SourceProtCtrl = DMA_SourcePrivileged;
	dma.DMA_DestProtCtrl = DMA_DestPrivileged;
	DMA_CtrlInit (DMA_Channel_ADC1, DMA_CTRL_DATA_PRIMARY, &dma);
	
	DMA_StructInit (&dma_ch);
	dma_ch.DMA_PriCtrlData = &dma;
	dma_ch.DMA_AltCtrlData = NULL;
	dma_ch.DMA_Priority = DMA_Priority_Default;
	dma_ch.DMA_ProtCtrl = DMA_AHB_Privileged;
	dma_ch.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
	dma_ch.DMA_UseBurst = DMA_BurstClear;	
	DMA_Init (DMA_Channel_ADC1, &dma_ch);
	
	DMA_Cmd (DMA_Channel_ADC1, ENABLE);
	
	
	ADC_DeInit();
	adc.ADC_SynchronousMode = ADC_SyncMode_Independent;
	adc.ADC_StartDelay = 0;
	adc.ADC_TempSensor = ADC_TEMP_SENSOR_Disable;
	adc.ADC_TempSensorAmplifier = ADC_TEMP_SENSOR_AMPLIFIER_Disable;
	adc.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Disable;
	adc.ADC_IntVRefConversion = ADC_VREF_CONVERSION_Disable;
	adc.ADC_IntVRefTrimming = 0;
	ADC_Init (&adc);
	
	ADCx_StructInit (&adcx);
	adcx.ADC_ClockSource = ADC_CLOCK_SOURCE_CPU;
	adcx.ADC_SamplingMode = ADC_SAMPLING_MODE_CICLIC_CONV;
	adcx.ADC_ChannelSwitching = numChannels > 1 ? ADC_CH_SWITCHING_Enable : ADC_CH_SWITCHING_Disable;
	adcx.ADC_ChannelNumber = ADC_CH_ADC2;
	adcx.ADC_Channels = ADC_CH_ADC2_MSK | ADC_CH_ADC3_MSK | ADC_CH_ADC4_MSK;
	adcx.ADC_LevelControl = ADC_LEVEL_CONTROL_Disable;
	adcx.ADC_LowLevel = 0;
	adcx.ADC_HighLevel = 0;
	adcx.ADC_VRefSource = ADC_VREF_SOURCE_INTERNAL;
	adcx.ADC_IntVRefSource = ADC_INT_VREF_SOURCE_INEXACT;
	adcx.ADC_Prescaler = ADC_CLK_div_8;
	adcx.ADC_DelayGo = 7;
	ADC1_Init (&adcx);
	
	ADC1_Cmd (ENABLE);
	ADC1_Start();
}

uint16_t adcGetChannel(uint8_t channel)
{
	return adcValues[adcConfig[channel].dmaIndex];
}
