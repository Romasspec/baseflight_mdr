#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include "MDR32F9Qx_config.h"
#include "core_cm3.h"

#include "utils.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RAD    (M_PI / 180.0f)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

//#include "core_cm3.h"

#define togle_PB7	MDR_PORTB->RXTX ^= PORT_Pin_7
#define togle_PD2	MDR_PORTD->RXTX ^= PORT_Pin_2
#define togle_PF4	MDR_PORTF->RXTX ^= PORT_Pin_4
#define togle_PF6	MDR_PORTF->RXTX ^= PORT_Pin_6

typedef uint16_t (*rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (*pidControllerFuncPtr)(void);                // pid controller function prototype

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SERIALRX = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
    FEATURE_POWERMETER = 1 << 12,
    FEATURE_VARIO = 1 << 13,
    FEATURE_3D = 1 << 14,
    FEATURE_FW_FAILSAFE_RTH = 1 << 15,
    FEATURE_SYNCPWM = 1 << 16,
    FEATURE_FASTPWM = 1 << 17,
} AvailableFeatures;

typedef enum {
	GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MTK_NMEA,
    GPS_MTK_BINARY,
    GPS_MAG_BINARY,
    GPS_HARDWARE_MAX = GPS_MAG_BINARY
} GPSHardware;

typedef enum {
    GPS_BAUD_115200 = 0,
    GPS_BAUD_57600,
    GPS_BAUD_38400,
    GPS_BAUD_19200,
    GPS_BAUD_9600,
    GPS_BAUD_MAX = GPS_BAUD_9600
} GPSBaudRates;

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6,
} AvailableSensors;

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_MPU6500 = 5,
    ACC_NONE = 6
} AccelSensors;

typedef enum CompassSensors {
    MAG_DEFAULT = 0,
    MAG_HMC5883L = 1,
    MAG_AK8975 = 2,
    MAG_NONE = 3
} CompassSensors;

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

typedef void (*sensorInitFuncPtr)(sensor_align_e align);   // sensor init prototype
typedef void (*sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef void (*serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app


typedef struct sensor_t {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;

// Target definitions (NAZE, CJMCU, ... are same as in Makefile
#if defined(NAZE)
// Afroflight32

#define BEEP_PORT   MDR_PORTB
#define BEEP_PIN    PORT_Pin_4 // PB4 (Buzzer)


#define GYRO
#define ACC
//#define MAG
//#define BARO
//#define GPS
//#define LEDRING
//#define SONAR
#define BUZZER
//#define LED0
//#define LED1
//#define INVERTER
#define MOTOR_PWM_RATE 400

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO) // | SENSOR_MAG
//#define SENSORS_SET (0)
#define I2C_DEVICE (I2CDEV_1)

#include "eeprom.h"
#include "drv_system.h"
#include "drv_i2c.h"
#include "drv_gpio.h"
#include "drv_adc.h"
#include "drv_mpu.h"
#include "drv_serial.h"
#include "drv_uart.h"
#include "drv_timer.h"
#include "drv_pwm.h"

#else
#error TARGET NOT DEFINED!
#endif /* all conditions */


// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              //digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 //digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  //digitalLo(LED0_GPIO, LED0_PIN);
#else
#define LED0_TOGGLE
#define LED0_OFF
#define LED0_ON
#endif

#ifdef LED1
#define LED1_TOGGLE              //digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 //digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  //digitalLo(LED1_GPIO, LED1_PIN);
#else
#define LED1_TOGGLE
#define LED1_OFF
#define LED1_ON
#endif

#ifdef BEEP_PORT
#define BEEP_TOGGLE              digitalToggle(BEEP_PORT, BEEP_PIN)
#define BEEP_OFF                 systemBeep(false)
#define BEEP_ON                  systemBeep(true)
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif

