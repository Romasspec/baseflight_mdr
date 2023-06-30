#include "board.h"
#include "mw.h"

flags_t f;
int16_t debug[4];
uint32_t currentTime = 0;
uint32_t previousTime = 0;
static int32_t errorGyroI[3] = { 0, 0, 0 };
static int32_t errorAngleI[2] = { 0, 0 };
int16_t headFreeModeHold;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

uint16_t vbat;                  // battery voltage in 0.1V steps
int32_t amperage;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhdrawn;              // milliampere hours drawn from the battery since start
//int16_t telemTemperature1;      // gyro sensor temperature

int16_t rcData[RC_CHANS];       // interval [1000;2000]
int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE
uint16_t rssi;                  // range: [0;1023]
rcReadRawDataPtr rcReadRawFunc = NULL;  // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

static void pidMultiWii(void);
static void pidRewrite(void);
static void mwArm(void);
static void mwDisarm(void);
pidControllerFuncPtr pid_controller = pidMultiWii; // which pid controller are we using, defaultMultiWii

// Battery monitoring stuff
uint8_t batteryCellCount = 3;       // cell count
uint16_t batteryWarningVoltage;     // slow buzzer after this one, recommended 80% of battery used. Time to land.
uint16_t batteryCriticalVoltage;    // annoying buzzer after this one, battery is going to be dead.

// Time of automatic disarm when "Don't spin the motors when armed" is enabled.
static uint32_t disarmTime = 0;

uint8_t dynP8[3], dynI8[3], dynD8[3];
uint8_t rcOptions[CHECKBOXITEMS];

int16_t axisPID[3];

// **********************
// GPS
// **********************
int32_t GPS_coord[2];
int32_t GPS_home[3];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
uint16_t GPS_altitude, GPS_speed;   // altitude in 0.1m and speed in 0.1m/s
uint8_t GPS_update = 0;             // it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[3] = { 0, 0, 0 }; // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0;     // degrees * 10

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}

void annexCode(void)
{
	static uint32_t calibratedAccTime;
	int32_t axis, prop1, prop2;
	int32_t tmp, tmp2;
	
	// vbat shit
    static uint8_t vbatTimer = 0;
    static int32_t vbatRaw = 0;
    static int32_t amperageRaw = 0;
    static int64_t mAhdrawnRaw = 0;
    static int32_t vbatCycleTime = 0;
	
	prop2 = 128;
	
	for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - mcfg.midrc), 500);
        if (axis != 2) {        // ROLL & PITCH
            if (cfg.deadband) {
                if (tmp > cfg.deadband) {
                    tmp -= cfg.deadband;
                } else {
                    tmp = 0;
                }
            }
		tmp2 = tmp / 100;
		rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
		prop1 = 100 - (uint16_t)cfg.rollPitchRate[axis] * tmp / 500;
		prop1 = (uint16_t)prop1 * prop2 / 100;
		} else {                // YAW
            if (cfg.yawdeadband) {
                if (tmp > cfg.yawdeadband) {
                    tmp -= cfg.yawdeadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -mcfg.yaw_control_direction;
            prop1 = 100 - (uint16_t)cfg.yawRate * abs(tmp) / 500;
        }
        dynP8[axis] = (uint16_t)cfg.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)cfg.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < mcfg.midrc) {
            rcCommand[axis] = -rcCommand[axis];
		}
    }
	
	tmp = constrain(rcData[THROTTLE], mcfg.mincheck, 2000);
    tmp = (uint32_t)(tmp - mcfg.mincheck) * 1000 / (2000 - mcfg.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
	
	if (f.HEADFREE_MODE) {
        float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }
	
	if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLE) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }
	
	if (feature(FEATURE_VBAT)) {
        vbatCycleTime += cycleTime;
        if (!(++vbatTimer % VBATFREQ)) {
            vbatRaw -= vbatRaw / 8;
            vbatRaw += adcGetChannel(ADC_BATTERY);
            vbat = batteryAdcToVoltage(vbatRaw / 8);

            if (mcfg.power_adc_channel > 0) {
                amperageRaw -= amperageRaw / 8;
                amperageRaw += adcGetChannel(ADC_EXTERNAL_CURRENT);
                amperage = currentSensorToCentiamps(amperageRaw / 8);
                mAhdrawnRaw += (amperage * vbatCycleTime) / 1000;
                mAhdrawn = mAhdrawnRaw / (3600 * 100);
                vbatCycleTime = 0;
            }

        }
        // Buzzers for low and critical battery levels
        if (vbat <= batteryCriticalVoltage) {
//            buzzer(BUZZER_BAT_CRIT_LOW);     // Critically low battery
		} else if (vbat <= batteryWarningVoltage) {
//            buzzer(BUZZER_BAT_LOW);     // low battery
		}
    }
    // update buzzer handler
//    buzzerUpdate();
	
	if ((calibratingA > 0 && sensors(SENSOR_ACC)) || (calibratingG > 0)) {      // Calibration phasis
        LED0_TOGGLE;
    } else {
        if (f.ACC_CALIBRATED)
            LED0_OFF;
        if (f.ARMED)
            LED0_ON;
#ifndef TELEMETRY
//        checkTelemetryState();
#endif
	}
	
	if ((int32_t)(currentTime - calibratedAccTime) >= 0) {
        if (!f.SMALL_ANGLE) {
            f.ACC_CALIBRATED = 0; // the multi uses ACC and is not calibrated or is too much inclinated
            LED0_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            f.ACC_CALIBRATED = 1;
        }
    }
	
	serialCom();
}

void loop(void)
{
	static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
	static uint8_t rcSticks;            // this hold sticks position for command combos
	static uint32_t rcTime = 0;
	uint8_t stTmp = 0;
	int i;
	static uint32_t loopTime;
	uint32_t auxState = 0;
	
	bool rcReady = false;
	bool isThrottleLow = false;

	if (((int32_t)(currentTime - rcTime) >= 0) || rcReady) {				// 50Hz or data driven
		rcReady = false;
		rcTime = currentTime + 20000;
		computeRC();
		
		// ------------------ STICKS COMMAND HANDLER --------------------
		// checking sticks positions
		
		for (i = 0; i < 4; i++){
			stTmp >>= 2;
			if (rcData[i] > mcfg.mincheck) {			
				stTmp |= 0x80;  // check for MIN
			}
			if (rcData[i] < mcfg.maxcheck) {
				stTmp |= 0x40;  // check for MAX
			}
		}
		
		if (stTmp == rcSticks) {
			if (rcDelayCommand < 250)
			rcDelayCommand++;
		}
		else {
			rcDelayCommand = 0;
		}
		rcSticks = stTmp;
		
//		// perform actions
//		if (feature(FEATURE_3D) && (rcData[THROTTLE] > (mcfg.midrc - mcfg.deadband3d_throttle) && rcData[THROTTLE] < (mcfg.midrc + mcfg.deadband3d_throttle)))
//		{
//			isThrottleLow = true;
//		}	else
		if (!feature(FEATURE_3D) && (rcData[THROTTLE] < mcfg.mincheck))	{
			isThrottleLow = true;
		}
		if (isThrottleLow) {
			errorGyroI[ROLL] = 0;
			errorGyroI[PITCH] = 0;
			errorGyroI[YAW] = 0;
			errorAngleI[ROLL] = 0;
			errorAngleI[PITCH] = 0;
		}

		if (rcDelayCommand == 20)	{
			if (f.ARMED) {								// actions during armed
				if (cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE))	{											// Disarm on throttle down + yaw
					mwDisarm();
				}
			}
			else {											// actions during not armed
				i = 0;
				// GYRO calibration
				if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
					calibratingG = CALIBRATING_GYRO_CYCLES;
					
#ifdef GPS
                    if (feature(FEATURE_GPS))
                        GPS_reset_home_position();
#endif
                    if (sensors(SENSOR_BARO))
                        calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
                    if (!sensors(SENSOR_MAG))
                        heading = 0; // reset heading to zero after gyro calibration
                    // Inflight ACC Calibration
				} else if (feature(FEATURE_INFLIGHT_ACC_CAL) && (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI)) {
//                    if (AccInflightCalibrationMeasurementDone) {        // trigger saving into eeprom after landing
//                        AccInflightCalibrationMeasurementDone = false;
//                        AccInflightCalibrationSavetoEEProm = true;
//                    } else {
//                        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
//                        if (AccInflightCalibrationArmed) {
//                            buzzer(BUZZER_ACC_CALIBRATION);
//                        } else {
//                            buzzer(BUZZER_ACC_CALIBRATION_FAIL);
//                        }
//                    }
                }
				
				// Multiple configuration profiles
				if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) {       		   	// ROLL left  -> Profile 1
					i = 1;
				}
				else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) {	  	 	  // PITCH up   -> Profile 2
					i = 2;
				}
				else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) {  				// ROLL right -> Profile 3
					i = 3;
				}
				
				if (i) {
					mcfg.current_profile = i - 1;
					writeEEPROM(0, false);
					blinkLED(2, 40, i);
					// TODO alarmArray[0] = i;
				}
				
				
				if (cfg.activate[BOXARM] == 0 && (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)) {			// Arm via YAW
					mwArm();
				}
				else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {																// Calibrating Acc	
					calibratingA = CALIBRATING_ACC_CYCLES;
				}
				else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE)	{																// Calibrating Mag
					f.CALIBRATE_MAG = 1;
				}	
									
				// Acc Trim
				i = 0;
				if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
					cfg.angleTrim[PITCH] += 2;
					i = 1;
				}
				else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
					cfg.angleTrim[PITCH] -= 2;
					i = 1;
				}
				else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
					cfg.angleTrim[ROLL] += 2;
					i = 1;
				}
				else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
					cfg.angleTrim[ROLL] -= 2;
					i = 1;
				}
				
				if (i) {
					writeEEPROM(1, true);
					rcDelayCommand = 0; // allow autorepetition
				}
			}
		}
		
		// Check AUX switches

		for (i = 0; i < core.numAuxChannels; i++) {
			auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
		}
		for (i = 0; i < CHECKBOXITEMS; i++) {
			rcOptions[i] = (auxState & cfg.activate[i]) > 0;
		}

		// note: if FAILSAFE is disable, failsafeCnt > 5 * FAILSAVE_DELAY is always false
		if ((rcOptions[BOXANGLE] ) && (sensors(SENSOR_ACC))) {
			// bumpless transfer to Level mode
			if (!f.ANGLE_MODE) {
				errorAngleI[ROLL] = 0;
				errorAngleI[PITCH] = 0;
				f.ANGLE_MODE = 1;
			}
			if (feature(FEATURE_FW_FAILSAFE_RTH)) {
//                if ((failsafeCnt > 5 * cfg.failsafe_delay) && sensors(SENSOR_GPS)) {
                   f.FW_FAILSAFE_RTH_ENABLE = 1;
//                }
            }
		} else {
			f.ANGLE_MODE = 0;   // failsafe support
			f.FW_FAILSAFE_RTH_ENABLE = 0;
		}		
		
		if (rcOptions[BOXHORIZON]) {
			f.ANGLE_MODE = 0;
			
			if (!f.HORIZON_MODE) {
				errorAngleI[ROLL] = 0;
				errorAngleI[PITCH] = 0;
				f.HORIZON_MODE = 1;
			}
		} else {
			f.HORIZON_MODE = 0;
		}

		if ((rcOptions[BOXARM]) == 0) {
			f.OK_TO_ARM = 1;
		}
		
		if (f.ANGLE_MODE || f.HORIZON_MODE) {
			LED1_ON;
		} else {
			LED1_OFF;
		}
		
#ifdef  MAG
		if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
			if (rcOptions[BOXMAG]) {
				if (!f.MAG_MODE) {
					f.MAG_MODE = 1;
					magHold = heading;
				}
			} else {
				f.MAG_MODE = 0;
			}
			
			if (rcOptions[BOXHEADFREE]) {
				if (!f.HEADFREE_MODE) {
					f.HEADFREE_MODE = 1;
				}
			} else {
				f.HEADFREE_MODE = 0;
			}
			
			if (rcOptions[BOXHEADADJ]) {
				headFreeModeHold = heading; // acquire new heading
			}
		}
#endif
		
		if (rcOptions[BOXPASSTHRU] && !f.FW_FAILSAFE_RTH_ENABLE) {
            f.PASSTHRU_MODE = 1;
        } else {
            f.PASSTHRU_MODE = 0;
        }
	
		// When armed and motors aren't spinning. Make warning beeps so that accidentally won't lose fingers...
		// Also disarm board after 5 sec so users without buzzer won't lose fingers.
		if (feature(FEATURE_MOTOR_STOP) && f.ARMED && !f.FIXED_WING) {
			if (isThrottleLow) {
				if (disarmTime == 0) {
					disarmTime = millis() + 1000 * mcfg.auto_disarm_board;
				}
				else if (disarmTime < millis() && mcfg.auto_disarm_board != 0) {
					mwDisarm();
				}
				
//				buzzer(BUZZER_ARMED);
			}
			else if (disarmTime != 0) {
				disarmTime = 0;
			}
		}		
	} else { 						// not rc loop
		static int taskOrder = 0;   // never call all function in the same loop, to avoid high delay spikes
		switch (taskOrder) {
		case 0:
			taskOrder++;

#ifdef MAG
			if (sensors(SENSOR_MAG) && Mag_getADC())
		break;
#endif
								
		case 1:
			taskOrder++;
#ifdef BARO
			if (sensors(SENSOR_BARO) && Baro_update())
		break;
#endif
		case 2:
			taskOrder++;
#ifdef BARO
			if (sensors(SENSOR_BARO) && getEstimatedAltitude())
		break;
#endif
		case 3:
			// if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
			// hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
			// change this based on available hardware
			taskOrder++;
#ifdef GPS
			if (feature(FEATURE_GPS)) {
			gpsThread();
		break;
			}
#endif
		case 4:
			taskOrder = 0;
#ifdef SONAR
			if (sensors(SENSOR_SONAR)) {
			Sonar_update();
			}
#endif
			if (feature(FEATURE_VARIO) && f.VARIO_MODE)
//				mwVario();
			break;
		}		
	}
	
	currentTime = micros();
	
	if (mcfg.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + mcfg.looptime;
		computeIMU();		
		
        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;
        // non IMU critical, temeperatur, serialcom
		
		annexCode();
		
#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
                int16_t dif = heading - magHold;
                if (dif <= -180)
                    dif += 360;
                if (dif >= +180)
                    dif -= 360;
                dif *= -mcfg.yaw_control_direction;
                if (f.SMALL_ANGLE)
                    rcCommand[YAW] -= dif * cfg.P8[PIDMAG] / 30;    // 18 deg
            } else
                magHold = heading;
        }
#endif
		// PID - note this is function pointer set by setPIDController()
        pid_controller();
		
		mixTable();
        writeServos();
        writeMotors();
		
	}
	
}

void computeRC(void)
{
	uint16_t capture;
	int i, chan;	
	static int16_t rcDataAverage[RC_CHANS][4];
	static int rcAverageIndex = 0;
	
	for (chan = 0; chan < mcfg.rc_channel_count; chan++)
	{		
		capture = rcReadRawFunc(chan);
		// validate input
		if (capture < PULSE_MIN || capture > PULSE_MAX)
		{
			capture = mcfg.midrc;
		}		
		rcDataAverage[chan][rcAverageIndex % 4] = capture;
		// clear this since we're not accessing it elsewhere. saves a temp var
		rcData[chan] = 0;
		for (i = 0; i < 4; i++)
		{
			rcData[chan] += rcDataAverage[chan][i];
		}		
		rcData[chan] /= 4;		
	}	
	rcAverageIndex++;	
}

static void mwArm(void)
{
	if (calibratingG == 0 && f.ACC_CALIBRATED)
	{
		if (!f.ARMED)        												  // arm now!
		{
			f.ARMED = 1;
//			buzzer(BUZZER_ARMING);
		}
	} else if (!f.ARMED) {
		blinkLED(2, 255, 1);
	}
}

static void mwDisarm(void)
{
	if (f.ARMED)
	{
		f.ARMED = 0;
	}
	
//  buzzer(BUZZER_DISARMING);												// Beep for inform about disarming
	
	if (disarmTime != 0)														// Reset disarm time so that it works next time we arm the board.
	{
		disarmTime = 0;
		}            
}

void setPIDController(int type)
{
    switch (type) {
        case 0:
        default:
            pid_controller = pidMultiWii;
            break;
        case 1:
            pid_controller = pidRewrite;
            break;
    }
}

static void pidMultiWii(void)
{
	int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    // **** PITCH & ROLL & YAW PID ****
    prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]
    for (axis = 0; axis < 3; axis++) {
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2) { // MODE relying on ACC
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)mcfg.max_angle_inclination), +mcfg.max_angle_inclination) - angle[axis] + cfg.angleTrim[axis];
            PTermACC = errorAngle * cfg.P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
            PTermACC = constrain(PTermACC, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12;
        }
        if (!f.ANGLE_MODE || f.HORIZON_MODE || axis == 2) { // MODE relying on GYRO or YAW axis
            error = (int32_t)rcCommand[axis] * 10 * 8 / cfg.P8[axis];
            error -= gyroData[axis];

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroData[axis]) > 640) || ((axis == YAW) && (abs(rcCommand[axis]) > 100)))
                errorGyroI[axis] = 0;
            ITermGYRO = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;
        }
        if (f.HORIZON_MODE && axis < 2) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            if (f.ANGLE_MODE && axis < 2) {
                PTerm = PTermACC;
                ITerm = ITermACC;
            } else {
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
        }

        PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = gyroData[axis] - lastGyro[axis];
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

static void pidRewrite(void)
{
	
}

uint16_t pwmReadRawRC(uint8_t chan)
{
    return pwmRead(mcfg.rcmap[chan]);
}


