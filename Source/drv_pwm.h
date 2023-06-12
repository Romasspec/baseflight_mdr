#pragma once

#define PULSE_1MS  				(1000)      // 1ms pulse width
#define PULSE_MIN   			(750)       // minimum PWM pulse width which is considered valid
#define PULSE_MAX   			(2250)      // maximum PWM pulse width which is considered valid
#define MAX_PPM_INPUTS 			8
#define MAX_MOTORS  			8
#define MAX_SERVOS  			8
#define MAX_PWM_INPUTS 			8

typedef struct drv_pwm_config_t {
    bool enableInput;
    bool usePPM;
    bool useUART;
    bool useSoftSerial;
    bool useServos;
    bool extraServos;    // configure additional 4 channels in PPM mode as servos, not motors
    bool airplane;       // fixed wing hardware config, lots of servos etc
    uint8_t pwmFilter;   // PWM ICFilter value for jittering input
    uint8_t adcChannel;  // steal one RC input for current sensor
    uint16_t motorPwmRate;
    uint16_t servoPwmRate;
    uint16_t idlePulse;  // PWM value to use when initializing the driver. set this to either PULSE_1MS (regular pwm),
    // some higher value (used by 3d mode), or 0, for brushed pwm drivers.
    uint16_t servoCenterPulse;
    uint16_t failsafeThreshold;
    bool syncPWM;
    bool fastPWM;

    // OUT parameters, filled by driver
    uint8_t numServos;
} drv_pwm_config_t;

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,    
    MAX_PORTS
};

static void pwmGPIOConfig(MDR_PORT_TypeDef *gpio, uint32_t pin, PORT_FUNC_TypeDef func);
void pwmICConfig(MDR_TIMER_TypeDef *tim, uint8_t channel, uint16_t polarity);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback);


static void pwmOCConfig(MDR_TIMER_TypeDef *tim, uint8_t channel, uint16_t value);

bool pwmInit(drv_pwm_config_t *init);
void pwmWriteMotor(uint8_t index, uint16_t value);
void pwmWriteServo(uint8_t index, uint16_t value);
uint16_t pwmRead(uint8_t channel);
