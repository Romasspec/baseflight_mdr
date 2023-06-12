#include "board.h"
#include "mw.h"

extern rcReadRawDataPtr rcReadRawFunc;
bool sensorEnable = false;

// receiver read function
extern uint16_t pwmReadRawRC(uint8_t chan);

core_t core;

int main(void)
{
	uint8_t i;
	drv_pwm_config_t pwm_params;
	drv_adc_config_t adc_params;
	bool sensorsOK = false;
	
	hardwareInit();	
	initEEPROM();
	
	delay(2000);
	
	checkFirstTime(false);
//	if (checkFirstTime(false))
//	{
//		UART_SendData (MDR_UART2, 0xFF);
//	}
	
//	readEEPROM();
	
//	uartTransmit ((uint8_t*) &mcfg, 1);
	
	// sleep for 100ms
	delay(100);

	activateConfig();

	i2cInit(I2C_DEVICE);

	// configure power ADC
	if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9 || mcfg.power_adc_channel == 5))
			adc_params.powerAdcChannel = mcfg.power_adc_channel;
	else {
			adc_params.powerAdcChannel = 0;
			mcfg.power_adc_channel = 0;
	}

	// configure rssi ADC
	if (mcfg.rssi_adc_channel > 0 && (mcfg.rssi_adc_channel == 1 || mcfg.rssi_adc_channel == 9 || mcfg.rssi_adc_channel == 5) && mcfg.rssi_adc_channel != mcfg.power_adc_channel)
			adc_params.rssiAdcChannel = mcfg.rssi_adc_channel;
	else {
			adc_params.rssiAdcChannel = 0;
			mcfg.rssi_adc_channel = 0;
	}
	
	adcInit (&adc_params);
	
	// Check battery type/voltage
	if (feature(FEATURE_VBAT))
	{
		batteryInit();
	}
	
	initBoardAlignment();
	// We have these sensors; SENSORS_SET defined in board.h depending on hardware platform
	sensorsSet(SENSORS_SET);
	// drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
	sensorsOK = sensorsAutodetect();
	
	// if gyro was not detected due to whatever reason, we give up now.
	sensorEnable = sensorsOK;
//	if (!sensorsOK) failureMode(3);
	
	LED1_ON;
	LED0_OFF;
	for (i = 0; i < 10; i++) {
			LED1_TOGGLE;
			LED0_TOGGLE;
			delay(25);
			BEEP_ON;
			delay(25);
			BEEP_OFF;
	}
	LED0_OFF;
	LED1_OFF;
	
	imuInit(); // Mag is initialized inside imuInit
	
	mixerInit();
	
	serialInit (mcfg.serial_baudrate);
	
	// when using airplane/wing mixer, servo/motor outputs are remapped
	if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_CUSTOM_PLANE)
	{
		pwm_params.airplane = true;
	} else
	{
		pwm_params.airplane = false;
	}
			
	pwm_params.useUART = feature(FEATURE_GPS) || feature(FEATURE_SERIALRX); // spektrum/sbus support uses UART too
	pwm_params.useSoftSerial = feature(FEATURE_SOFTSERIAL);
	pwm_params.usePPM = feature(FEATURE_PPM);
	pwm_params.enableInput = !feature(FEATURE_SERIALRX); // disable inputs if using spektrum
	pwm_params.useServos = core.useServo;
	pwm_params.motorPwmRate = mcfg.motor_pwm_rate;
	pwm_params.servoPwmRate = mcfg.servo_pwm_rate;
	pwm_params.pwmFilter = mcfg.pwm_filter;
	pwm_params.idlePulse = PULSE_1MS; // standard PWM for brushless ESC (default, overridden below)
	if (feature(FEATURE_3D))
        pwm_params.idlePulse = mcfg.neutral3d;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors
	pwm_params.syncPWM = feature(FEATURE_SYNCPWM);
	pwm_params.fastPWM = feature(FEATURE_FASTPWM);
	pwm_params.servoCenterPulse = mcfg.midrc;
	pwm_params.failsafeThreshold = cfg.failsafe_detect_threshold;
	
	pwmInit(&pwm_params);
	core.numServos = pwm_params.numServos;
	
	for (i = 0; i < RC_CHANS; i++)
	{
		rcData[i] = 1502;
	}

	rcReadRawFunc = pwmReadRawRC;
	core.numRCChannels = MAX_PWM_INPUTS;

#ifdef GPS
	gpsInit(mcfg.gps_baudrate);
#endif
	
	if (feature(FEATURE_PPM))
	{
		core.numRCChannels = MAX_PPM_INPUTS;
	}

	core.numAuxChannels = constrain ((mcfg.rc_channel_count - 4), 4, 8);
	
	if (feature(FEATURE_TELEMETRY))
	{
//		initTelemetry();
	}	
	
	previousTime = micros();
	
	uartTransmit((uint8_t*)(&previousTime), 4);
	calibratingG = CALIBRATING_GYRO_CYCLES;
	
	f.SMALL_ANGLE = 1;
	
	
	while (1)
	{	
		loop();
//		RXcan ( (uint32_t *)&mcfg);		
	}
}

void HardFault_Handler(void)
{
    // fall out of the sky
    writeAllMotors(mcfg.mincommand);
    while (1);
}
