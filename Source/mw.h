#pragma once
#include <stdint.h>
#include "can.h"
#include "uart.h"

#define  VERSION  231


//#define LAT  0
//#define LON  1
//#define ALT  2

enum {
	LAT = 0,
	LON,
	ALT
};

#define RC_CHANS    (6)

typedef enum MultiType {
		MULTITYPE_TRI = 1,
		MULTITYPE_QUADP,
		MULTITYPE_QUADX,
		MULTITYPE_BI,
		MULTITYPE_CUSTOM,          // no current GUI displays this
		MULTITYPE_AIRPLANE,
		MULTITYPE_FLYING_WING,
		MULTITYPE_CUSTOM_PLANE,
		MULTITYPE_LAST = 25
} MultiType;

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1
};

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
};

typedef struct {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

//enum {
//    BOXARM = 0,
//    BOXANGLE,
//    BOXHORIZON,
//    BOXBARO,    
//    BOXMAG,
//		BOXHEADFREE,
//    BOXHEADADJ,
//    CHECKBOXITEMS
//};

enum {
    BOXARM = 0,
    BOXANGLE,
    BOXHORIZON,
    BOXBARO,
    BOXVARIO,
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLEDLOW,
    BOXLLIGHTS,
    BOXCALIB,
    BOXGOV,
    BOXOSD,
    BOXTELEMETRY,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXGCRUISE,
    CHECKBOXITEMS
};

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct config_t {
	uint8_t pidController;                  // 0 = multiwii original, 1 = rewrite from http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671
    uint8_t P8[PIDITEMS];
    uint8_t I8[PIDITEMS];
    uint8_t D8[PIDITEMS];
	
	uint8_t rcRate8;
    uint8_t rcExpo8;
	
	uint8_t rollPitchRate[2];
    uint8_t yawRate;
	
	int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
    int16_t angleTrim[2];                   // accelerometer trim
	
	// sensor-related stuff
	uint8_t acc_lpf_factor;                 // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
	uint8_t accz_deadband;                  // set the acc deadband for z-Axis, this ignores small accelerations
    uint8_t accxy_deadband;                 // set the acc deadband for xy-Axis
	float accz_lpf_cutoff;                  // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
	uint8_t small_angle;                    // what is considered a safe angle for arming
	uint8_t acc_unarmedcal;                 // turn automatic acc compensation on/off
	
	uint32_t activate[CHECKBOXITEMS];       // activate switches

    // Radio/ESC-related configuration
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yawdeadband;                    // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
	uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
		
} config_t;

typedef struct master_t {
	uint8_t version;
	uint16_t size;
	uint8_t magic_be;                       // magic number, should be 0xBE
	
	uint8_t mixerConfiguration;
	uint32_t enabledFeatures;
	uint16_t looptime;                      // imu loop time in us
	
	// motor/esc/servo related stuff
	uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
	uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
	uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
	uint16_t motor_pwm_rate;                // The update rate of motor outputs (50-498Hz)
	uint16_t servo_pwm_rate;                // The update rate of servo outputs (50-498Hz)
	uint8_t pwm_filter;                     // Hardware filter for incoming PWM pulses (larger = more filtering)
	
	// global sensor-related stuff
	sensor_align_e gyro_align;              // gyro alignment
	sensor_align_e acc_align;               // acc alignment
	sensor_align_e mag_align;               // mag alignment
	int16_t board_align_roll;               // board alignment correction in roll (deg)
	int16_t board_align_pitch;              // board alignment correction in pitch (deg)
	int16_t board_align_yaw;                // board alignment correction in yaw (deg)
	int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)
	uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
	uint16_t gyro_lpf;                      // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
	uint16_t gyro_cmpf_factor;              // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint16_t gyro_cmpfm_factor;             // Set the Gyro Weight for Gyro/Magnetometer complementary filter. Increasing this value would reduce and delay Magnetometer influence on the output of the filter
	uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
    int16_t accZero[3];
    int16_t magZero[3];
	
	// Safety features
	uint8_t auto_disarm_board;              // Disarm board when motors not spinning at armed enabled (0 = disabled, 1 - 60 seconds when to automatically disarm)
	
	// Battery/ADC stuff
	uint8_t vbatscale;                      // adjust this to match battery voltage to reported value
	uint8_t vbatmaxcellvoltage;             // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
	uint8_t vbatmincellvoltage;             // minimum voltage per cell, this triggers FASTER battery out alarm, in 0.1V units, default is 33 (3.3V)
	uint8_t vbatwarningcellvoltage;         // minimum voltage per cell, this triggers SLOWER battery out alarm, in 0.1V units, default is 35 (3.5V)
	uint8_t power_adc_channel;              // which channel is used for current sensor. Right now, only 3 places are supported: RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9), ADC_EXTERNAL_PAD (Rev5 only, = 5), 0 to disable

	
	// Radio/ESC-related configuration
	uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
	uint8_t rcmap[RC_CHANS];                // mapping of radio channels to internal RPYTA+ order
	uint16_t mincheck;                      // minimum rc end
	uint16_t maxcheck;                      // maximum rc end
	uint8_t rssi_adc_channel;               // Read analog-rssi from RC-filter (RSSI-PWM to RSSI-Analog), RC_CH2 (unused when in CPPM mode, = 1), RC_CH8 (last channel in PWM mode, = 9), ADC_EXTERNAL_PAD (Rev5 only, = 5), 0 to disable (disabled if rssi_aux_channel > 0 or rssi_adc_channel == power_adc_channel)
	uint8_t rc_channel_count;               // total number of incoming RC channels that should be processed, range (8...18), default is 8
	
	// gps-related stuff
	uint8_t gps_type;                       // See GPSHardware enum.
	int8_t gps_baudrate;                    // See GPSBaudRates enum.
	
	
	
	uint32_t serial_baudrate;               // primary serial (MSP) port baudrate
	
	config_t profile[3];										// 3 separate profiles
	uint8_t current_profile;                // currently loaded profile
	
	uint8_t magic_ef;                       // magic number, should be 0xEF
	uint8_t chk;                            // XOR checksum
} master_t;

// Core runtime settings
typedef struct core_t {
    serialPort_t *mainport;
    serialPort_t *flexport;
    serialPort_t *gpsport;
    serialPort_t *telemport;
    serialPort_t *rcvrport;
    uint8_t numRCChannels;                  // number of rc channels as reported by current input driver
    uint8_t numAuxChannels;
    bool useServo;                          // feature SERVO_TILT or wing/airplane mixers will enable this
    uint8_t numServos;                      // how many total hardware servos we have. used by mixer
} core_t;

typedef struct flags_t {
    uint8_t OK_TO_ARM;
    uint8_t ARMED;
    uint8_t ACC_CALIBRATED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLE;
    uint8_t CALIBRATE_MAG;
    uint8_t VARIO_MODE;
    uint8_t FIXED_WING;                     // set when in flying_wing or airplane mode. currently used by althold selection code
    uint8_t MOTORS_STOPPED;
    uint8_t FW_FAILSAFE_RTH_ENABLE;
    uint8_t CLIMBOUT_FW;
    uint8_t CRUISE_MODE;
} flags_t;

#define PITCH_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12

extern int16_t gyroZero[3];
extern int16_t gyroData[3];
extern int16_t angle[2];
extern uint8_t rcOptions[CHECKBOXITEMS];

extern uint16_t acc_1G;
extern uint32_t currentTime;

extern master_t mcfg;
extern config_t cfg;
extern sensor_t acc;
extern sensor_t gyro;
extern core_t core;
extern int16_t rcData[RC_CHANS];
extern uint32_t previousTime;
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern flags_t f;
extern int16_t heading, magHold;
extern int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];
extern uint16_t cycleTime;
extern int32_t EstAlt;

// GPS stuff
extern int32_t  GPS_coord[2];
extern int32_t 	GPS_home[3];
extern uint8_t  GPS_numSat;
extern uint16_t GPS_distanceToHome;                          // distance to home or hold point in meters
extern int16_t  GPS_directionToHome;                         // direction to home or hol point in degrees
extern uint16_t GPS_altitude, GPS_speed;                     // altitude in 0.1m and speed in 0.1m/s
extern uint8_t  GPS_update;                                  // it's a binary toogle to distinct a GPS position update
extern uint16_t GPS_ground_course;                           // degrees*10

// main
void setPIDController(int type);
void loop(void);

// rx
void computeRC(void);

// IMU
void imuInit(void);
void computeIMU(void);

// sensors
void batteryInit(void);
bool sensorsAutodetect(void);
void Mag_init(void);
void Gyro_getADC(void);
void ACC_getADC(void);

// Output
void mixerInit(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);

// Serial
void serialInit(uint32_t baudrate);
void serialCom(void);

// Config
void initEEPROM(void);
uint8_t checkFirstTime(bool reset);
void writeEEPROM(uint8_t b, uint8_t updateProfile);
void readEEPROM(void);
void activateConfig(void);
bool feature(uint32_t mask);
void sensorsSet(uint32_t mask);
bool sensors(uint32_t mask);

// gps
void gpsInit(uint8_t baudrate);
void gpsThread(void);
void gpsSetPIDs(void);
void GPS_reset_home_position(void);

