#include "board.h"
extern uint16_t debug[4];

typedef struct adc_config_t {
    uint8_t adcChannel;         // ADC1_INxx channel number
    uint8_t dmaIndex;           // index into DMA buffer in case of sparse channels
} adc_config_t;

static adc_config_t adcConfig[ADC_CHANNEL_MAX];
static volatile uint32_t adcValues[ADC_CHANNEL_MAX];
DMA_CtrlDataInitTypeDef dma;
DMA_ChannelInitTypeDef dma_ch;

void adcInit(drv_adc_config_t *init)
{	
	ADC_InitTypeDef adc;
	ADCx_InitTypeDef adcx;
	
	int numChannels = 1;

	// configure always-present battery index (ADC2)
	adcConfig[ADC_BATTERY].adcChannel = ADC_CH_ADC2;
	adcConfig[ADC_BATTERY].dmaIndex = numChannels - 1;
	
	// optional ADC4 input on rev.5 hardware
	if (1) {
			numChannels++;
			adcConfig[ADC_EXTERNAL_PAD].adcChannel = ADC_CH_ADC4;
			adcConfig[ADC_EXTERNAL_PAD].dmaIndex = numChannels - 1;
	}
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
	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_DMA | RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2, ENABLE); // bug DMA, CLK ON SSP1 and SSP2
	
	DMA_DeInit ();	
	dma.DMA_SourceBaseAddr = (uint32_t)(&(MDR_ADC->ADC1_RESULT));
	dma.DMA_DestBaseAddr = (uint32_t) (&(adcValues[0]));
	dma.DMA_SourceIncSize = DMA_SourceIncNo;
	dma.DMA_DestIncSize = numChannels > 1 ? DMA_DestIncWord : DMA_DestIncNo;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma.DMA_Mode = DMA_Mode_Basic;
	dma.DMA_CycleSize = ADC_CHANNEL_MAX;
	dma.DMA_NumContinuous = DMA_Transfers_1;
	dma.DMA_SourceProtCtrl = DMA_SourcePrivileged;
	dma.DMA_DestProtCtrl = DMA_DestPrivileged;
	
	DMA_StructInit (&dma_ch);
	dma_ch.DMA_PriCtrlData = &dma;
	dma_ch.DMA_AltCtrlData = NULL;
	dma_ch.DMA_Priority = DMA_Priority_Default;
	dma_ch.DMA_ProtCtrl = DMA_AHB_Privileged;
	dma_ch.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
	dma_ch.DMA_UseBurst = DMA_BurstClear;	
	DMA_Init (DMA_Channel_ADC1, &dma_ch);
	
	DMA_Cmd (DMA_Channel_ADC1, ENABLE);
	NVIC_EnableIRQ(DMA_IRQn);
	
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
	adcx.ADC_Channels = ADC_CH_ADC2_MSK | ADC_CH_ADC3_MSK | ADC_CH_ADC4_MSK | ADC_CH_ADC7_MSK;
	adcx.ADC_LevelControl = ADC_LEVEL_CONTROL_Disable;
	adcx.ADC_LowLevel = 0;
	adcx.ADC_HighLevel = 0;
	adcx.ADC_VRefSource = ADC_VREF_SOURCE_INTERNAL;
	adcx.ADC_IntVRefSource = ADC_INT_VREF_SOURCE_INEXACT;
	adcx.ADC_Prescaler = ADC_CLK_div_256;
	adcx.ADC_DelayGo = 0;
	ADC1_Init (&adcx);
	
	ADC1_Cmd (ENABLE);
	ADC1_Start();
}

uint16_t adcGetChannel(uint8_t channel)
{
	uint8_t i;
	for (i=0; i < 4; i++) {
		if (((adcValues[adcConfig[i].dmaIndex]>>16)&0x1F) == adcConfig[channel].adcChannel) {			
			return (uint16_t) (adcValues[adcConfig[i].dmaIndex]);
		}
	}
	return 0;
}

void DMA_IRQHandler (void)
{
	DMA_CtrlDataTypeDef *ptr_adc_dma;										// Указатель на адрес непосредственно на таблицу структур, которую использует DMA
																			// Вычисляем адрес структуры конкретного канала DMA
	ptr_adc_dma = (DMA_CtrlDataTypeDef*) (MDR_DMA->CTRL_BASE_PTR + (DMA_Channel_ADC1 * sizeof(DMA_CtrlDataTypeDef)));
	
	if ((ptr_adc_dma->DMA_Control & 0x3FF0) == 0) {							// тут накладываем маску на поле числа передач DMA и если передачи кончились
		ptr_adc_dma->DMA_Control |= ((ADC_CHANNEL_MAX-1) << 4)|(1 << 0);	// инициализируем колличество передач заново а так же устанавливаем режим работы DMA
		MDR_DMA->CHNL_ENABLE_SET = (1 << DMA_Channel_ADC1);					// включаем канал, т.к. после окончания всех передач он отключается
		togle_PF6;
	}
}
