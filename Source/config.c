#include "board.h"
#include "mw.h"

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 4
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
// if sizeof(mcfg) is over this number, compile-time error will occur. so, need to add another page to config data.
#define CONFIG_SIZE                     (FLASH_PAGE_SIZE)


master_t mcfg;  // master config struct with data independent from profiles
config_t cfg;   // profile config struct

static const uint8_t EEPROM_CONF_VERSION = 76;
static uint32_t enabledSensors = 0;
static const uint32_t FLASH_WRITE_ADDR = 0x08000000;// + (FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - (CONFIG_SIZE / 1024)));

static void resetConf(void);

void initEEPROM (void)
{
	ct_assert(sizeof(mcfg) < CONFIG_SIZE);
}

static uint8_t validEEPROM(void)
{
	const master_t *temp = (const master_t *) &mcfg;
	const uint8_t *p;
	uint8_t chk = 0;
	
	// check version number
	if (EEPROM_CONF_VERSION != temp->version)
	{
		return 0;
	}
	
	// check size and magic numbers
	if(temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
	{
		return 0;
	}
	
	// verify integrity of temporary copy
	for(p = (const uint8_t *) temp; p < ((const uint8_t *) temp + sizeof(master_t)); p++)
	{
		chk ^= *p;
	}
	
	// checksum failed
	if (chk != 0)
	{
		return 0;
	}
	
	// looks good, let's roll!
    return 1;
}

uint8_t checkFirstTime(bool reset)
{

	// check the EEPROM integrity before resetting values
	readEEPROM();
	if (!validEEPROM() || reset)
	{

		resetConf();		
		writeEEPROM(0, false);
		return 1;
	}
	return 0;
}

// Default settings
static void resetConf(void)
{
	 // Clear all configuration
	memset (&mcfg, 0, sizeof(master_t));
	memset (&cfg, 0, sizeof(config_t));
	
	mcfg.version = EEPROM_CONF_VERSION;
	mcfg.mixerConfiguration = MULTITYPE_QUADX;
	
	// global settings
	mcfg.gyro_cmpf_factor = 600;    // default MWC
    mcfg.gyro_cmpfm_factor = 250;   // default MWC
	mcfg.gyro_lpf = 42;             // supported by all gyro drivers now. In case of ST gyro, will default to 32Hz instead
	
	mcfg.gyro_align = ALIGN_DEFAULT;
	mcfg.acc_align = ALIGN_DEFAULT;
	mcfg.mag_align = ALIGN_DEFAULT;
	mcfg.board_align_roll = 0;
	mcfg.board_align_pitch = 0;
	mcfg.board_align_yaw = 0;
	mcfg.accZero[0] = 0;
    mcfg.accZero[1] = 0;
    mcfg.accZero[2] = 0;
	mcfg.yaw_control_direction = 1;
	// Motor/ESC/Servo
	mcfg.minthrottle = 1150;
	mcfg.maxthrottle = 1850;
	mcfg.mincommand = 1000;
	mcfg.motor_pwm_rate = MOTOR_PWM_RATE;
	mcfg.servo_pwm_rate = 50;
	mcfg.auto_disarm_board = 5; // auto disarm after 5 sec if motors not started or disarmed
	// gps/nav stuff
    mcfg.gps_type = GPS_NMEA;
    mcfg.gps_baudrate = GPS_BAUD_9600;//GPS_BAUD_115200;
	
	mcfg.vbatscale = 110;
	mcfg.vbatmaxcellvoltage = 43;
	mcfg.vbatmincellvoltage = 33;
	mcfg.vbatwarningcellvoltage = 35;
	mcfg.power_adc_channel = 0;
	mcfg.midrc = 1500;
	mcfg.mincheck = 1100;
	mcfg.maxcheck = 1900;	
	mcfg.acc_hardware = ACC_MPU6050;     // default/autodetect
	mcfg.rc_channel_count = 8;
	mcfg.looptime = 3500;
	
	
	// serial (USART2) baudrate
	mcfg.serial_baudrate = 115200;
	
	cfg.pidController = 0;
	cfg.P8[ROLL] = 40;
	cfg.I8[ROLL] = 30;
	cfg.D8[ROLL] = 20;
	cfg.P8[PITCH] = 40;
	cfg.I8[PITCH] = 30;
	cfg.D8[PITCH] = 20;
	cfg.P8[YAW] = 85;
	cfg.I8[YAW] = 45;
	cfg.D8[YAW] = 0;
	cfg.P8[PIDLEVEL] = 90;
	cfg.I8[PIDLEVEL] = 10;
	cfg.D8[PIDLEVEL] = 100;
	cfg.P8[PIDMAG] = 40;
	cfg.rcRate8 = 90;
	cfg.rcExpo8 = 65;
	cfg.yawRate = 0;
	cfg.mag_declination = 0;    // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
	cfg.angleTrim[0] = 0;
	cfg.angleTrim[1] = 0;
	cfg.deadband = 0;
	cfg.yawdeadband = 0;
	cfg.small_angle = 25;
	cfg.accz_lpf_cutoff = 5.0f;
	cfg.acc_lpf_factor = 4;
	cfg.accz_deadband = 40;
    cfg.accxy_deadband = 40;
	cfg.acc_unarmedcal = 1;
	
	cfg.throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv
	
	for(uint8_t i=0; i<3; i++)
	{
		memcpy(&mcfg.profile[i], &cfg, sizeof(config_t));
	}	
}

void writeEEPROM (uint8_t b, uint8_t updateProfile)
{
	uint8_t chk = 0;
	const uint8_t *p;
	
	// prepare checksum/version constants
	mcfg.version = EEPROM_CONF_VERSION;
	mcfg.size = sizeof(master_t);
	mcfg.magic_be = 0xBE;
	mcfg.magic_ef = 0xEF;
	mcfg.chk = 0;
	
	// recalculate checksum before writing
	for (p = (const uint8_t *)&mcfg; p < ((const uint8_t *)&mcfg + sizeof(master_t)); p++)
	{
		chk ^= *p;
	}
	mcfg.chk = chk;
	
	FLASH_ErasePage (FLASH_WRITE_ADDR, EEPROM_Info_Bank_Select);
	for (uint16_t i=0; i < sizeof(master_t); i+=4)
	{		
		//FLASH_ProgramByte (FLASH_WRITE_ADDR+i, *((uint8_t *)&mcfg + i), EEPROM_Info_Bank_Select);
		FLASH_ProgramWord (FLASH_WRITE_ADDR + i, *(uint32_t*)((uint8_t*)&mcfg + i), EEPROM_Info_Bank_Select);
	}
	 
	if (!validEEPROM())
	{
//		failureMode(10);
	}
	
	// re-read written data
//    loadAndActivateConfig();
    if (b)
		{
//			blinkLED(15, 20, 1);
		}        
}

void readEEPROM(void)
{
	// Sanity check
   if (!validEEPROM())
	 {
//		 failureMode(10);
	 }        

   // Read flash	 
	 memset (&mcfg, 0xBB, sizeof (master_t));
	 
	 for (uint16_t i=0; i < sizeof (master_t); i ++)
	 {		
		 FLASH_ReadByte (FLASH_WRITE_ADDR + i, (uint8_t*)&mcfg + i, EEPROM_Info_Bank_Select);		 
	 } 
}

bool sensors(uint32_t mask)
{
	return enabledSensors & mask;
}

void activateConfig(void)
{
	setPIDController(cfg.pidController);
#ifdef GPS
	gpsSetPIDs();
#endif
}

bool feature(uint32_t mask)
{
    return mcfg.enabledFeatures & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

