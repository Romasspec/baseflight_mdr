#include "board.h"
#include "mw.h"

static uint8_t numberMotor = 0;
static uint8_t numberRules = 0;
int16_t motor[MAX_MOTORS];
int16_t motor_disarmed[MAX_MOTORS];
int16_t servo[MAX_SERVOS] = { 1500, 1500, 1500, 1500};

static motorMixer_t currentMixer[MAX_MOTORS];
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];

static const motorMixer_t mixerTri[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};

static const motorMixer_t mixerBi[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};

// Keep this synced with MultiType struct in mw.h!
const mixer_t mixers[] = {
//    Mo Se Mixtable
    { 0, 0, NULL },                // entry 0
    { 3, 1, mixerTri },            // MULTITYPE_TRI
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
    { 2, 1, mixerBi },             // MULTITYPE_BI
    { 0, 1, NULL },                // * MULTITYPE_GIMBAL
	{ 0, 0, NULL },                // MULTITYPE_CUSTOM
};

// mixer rule format servo, input, rate, speed, min, max, box
static const servoMixer_t servoMixerAirplane[] = {
    { 3, INPUT_ROLL,  100, 0, 0, 100, 0 },
    { 4, INPUT_ROLL,  100, 0, 0, 100, 0 },
    { 5, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 6, INPUT_PITCH, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerFlyingWing[] = {
    { 3, INPUT_ROLL,  100, 0, 0, 100, 0 },
    { 3, INPUT_PITCH, 100, 0, 0, 100, 0 },
    { 4, INPUT_ROLL,  100, 0, 0, 100, 0 },
    { 4, INPUT_PITCH, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerBI[] = {
    { 4, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 4, INPUT_PITCH, 100, 0, 0, 100, 0 },
    { 5, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 5, INPUT_PITCH, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerTri[] = {
    { 5, INPUT_YAW,   100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerDual[] = {
    { 4, INPUT_PITCH, 100, 0, 0, 100, 0 },
    { 5, INPUT_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerSingle[] = {
    { 3, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 3, INPUT_PITCH, 100, 0, 0, 100, 0 },
    { 4, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 4, INPUT_PITCH, 100, 0, 0, 100, 0 },
    { 5, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 5, INPUT_ROLL,  100, 0, 0, 100, 0 },
    { 6, INPUT_YAW,   100, 0, 0, 100, 0 },
    { 6, INPUT_ROLL,  100, 0, 0, 100, 0 },
};

const mixerRules_t servoMixers[] = {
    { 0, NULL },                // entry 0
    { 1, servoMixerTri },       // MULTITYPE_TRI
    { 0, NULL },                // MULTITYPE_QUADP
    { 0, NULL },                // MULTITYPE_QUADX
    { 4, servoMixerBI },        // MULTITYPE_BI
    { 0, NULL },                // * MULTITYPE_GIMBAL
    { 0, NULL },                // MULTITYPE_Y6
    { 0, NULL },                // MULTITYPE_HEX6
    { 4, servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, NULL },                // MULTITYPE_Y4
    { 0, NULL },                // MULTITYPE_HEX6X
    { 0, NULL },                // MULTITYPE_OCTOX8
    { 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, NULL },                // MULTITYPE_OCTOFLATX
    { 4, servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { 0, NULL },                // * MULTITYPE_HELI_120_CCPM
    { 0, NULL },                // * MULTITYPE_HELI_90_DEG
    { 0, NULL },                // MULTITYPE_VTAIL4
    { 0, NULL },                // MULTITYPE_HEX6H
    { 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { 2, servoMixerDual },      // MULTITYPE_DUALCOPTER
    { 8, servoMixerSingle },    // MULTITYPE_SINGLECOPTER
    { 0, NULL },                // MULTITYPE_ATAIL4
    { 0, NULL },                // MULTITYPE_CUSTOM
    { 0, NULL },                // MULTITYPE_CUSTOM_PLANE
};

void loadCustomServoMixer(void)
{
    uint8_t i;

    // reset settings
    numberRules = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (mcfg.customServoMixer[i].rate == 0)
            break;
        currentServoMixer[i] = mcfg.customServoMixer[i];
        numberRules++;
    }
}

void mixerInit(void)
{

	int i;
	// enable servos for mixes that require them. note, this shifts motor counts.
	core.useServo = mixers[mcfg.mixerConfiguration].useServo;
	// if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (feature(FEATURE_SERVO_TILT))
        core.useServo = 1;

	if (mcfg.mixerConfiguration == MULTITYPE_CUSTOM) {
		// load custom mixer into currentMixer
        for (i = 0; i < MAX_MOTORS; i++) {
            // check if done
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            currentMixer[i] = mcfg.customMixer[i];
            numberMotor++;
        }
	} else {
        numberMotor = mixers[mcfg.mixerConfiguration].numberMotor;
        // copy motor-based mixers
        if (mixers[mcfg.mixerConfiguration].motor) {
            for (i = 0; i < numberMotor; i++)
                currentMixer[i] = mixers[mcfg.mixerConfiguration].motor[i];
        }
    }
	
	if (core.useServo) {
        numberRules = servoMixers[mcfg.mixerConfiguration].numberRules;
        if (servoMixers[mcfg.mixerConfiguration].rule) {
            for (i = 0; i < numberRules; i++)
                currentServoMixer[i] = servoMixers[mcfg.mixerConfiguration].rule[i];
        }
    }
	
	// in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_3D)) {
        if (numberMotor > 1) {
            for (i = 0; i < numberMotor; i++) {
                currentMixer[i].pitch *= 0.5f;
                currentMixer[i].roll *= 0.5f;
                currentMixer[i].yaw *= 0.5f;
            }
        }
    }
	
	// set flag that we're on something with wings
    if (mcfg.mixerConfiguration == MULTITYPE_FLYING_WING || mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_CUSTOM_PLANE) {
        f.FIXED_WING = 1;

        if (mcfg.mixerConfiguration == MULTITYPE_CUSTOM_PLANE)
            loadCustomServoMixer();
    } else
        f.FIXED_WING = 0;

    mixerResetMotors();
	
}

void mixerResetMotors(void)
{
    int i;
    // set disarmed motor values
    for (i = 0; i < MAX_MOTORS; i++)
        motor_disarmed[i] = feature(FEATURE_3D) ? mcfg.neutral3d : mcfg.mincommand;
}

void writeServos(void)
{
	if (!core.useServo)
        return;
	
	switch (mcfg.mixerConfiguration) {
        case MULTITYPE_BI:
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            break;		

        case MULTITYPE_FLYING_WING:
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            break;

        case MULTITYPE_AIRPLANE:        
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            pwmWriteServo(2, servo[2]);
            pwmWriteServo(3, servo[3]);
            break;
		
		case MULTITYPE_CUSTOM_PLANE:
            pwmWriteServo(0, servo[0]);
            pwmWriteServo(1, servo[1]);
            pwmWriteServo(2, servo[2]);
            pwmWriteServo(3, servo[3]);
            if (feature(FEATURE_PPM)) {
                pwmWriteServo(0, servo[0]);
                pwmWriteServo(1, servo[1]);
                pwmWriteServo(2, servo[2]);
                pwmWriteServo(3, servo[3]);
            }
            break;

        default:
            // Two servos for SERVO_TILT, if enabled
            if (feature(FEATURE_SERVO_TILT)) {
                pwmWriteServo(0, servo[0]);
                pwmWriteServo(1, servo[1]);
            }
            break;
	}
}

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < numberMotor; i++) {
        pwmWriteMotor(i, motor[i]);
	}
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;

    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++) {
        motor[i] = mc;
	}
    writeMotors();
}

void mixTable(void)
{
	uint32_t i;
	int16_t maxMotor;
	
	if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }
	
	// motors for non-servo mixes
    if (numberMotor > 1) {
		for (i = 0; i < numberMotor; i++) {
			motor[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + -cfg.yaw_direction * axisPID[YAW] * currentMixer[i].yaw;
			if (f.FIXED_WING) { // vector_thrust handeling
				if (cfg.fw_vector_thrust) {
                    if (f.PASSTHRU_MODE)
                        motor[i] = rcCommand[THROTTLE] - rcCommand[YAW] * (i - 0.5f);
                } else { // Override mixerVectorThrust
                    motor[i] = rcCommand[THROTTLE];
                }				
			}
		}		
	}
	
	// airplane / servo mixes
    switch (mcfg.mixerConfiguration) {
        case MULTITYPE_CUSTOM_PLANE:
        case MULTITYPE_FLYING_WING:
        case MULTITYPE_AIRPLANE:
        case MULTITYPE_BI:
        case MULTITYPE_TRI:
//        case MULTITYPE_DUALCOPTER:
//        case MULTITYPE_SINGLECOPTER:
//            servoMixer();
            break;
//        case MULTITYPE_GIMBAL:
//            servo[0] = (((int32_t)cfg.servoConf[0].rate * angle[PITCH]) / 50) + servoMiddle(0);
//            servo[1] = (((int32_t)cfg.servoConf[1].rate * angle[ROLL]) / 50) + servoMiddle(1);
//            break;
    }
	
	 // constrain servos
    for (i = 0; i < MAX_SERVOS; i++) {
        servo[i] = constrain(servo[i], cfg.servoConf[i].min, cfg.servoConf[i].max); // limit the values
	}
	
	maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++) {
        if (motor[i] > maxMotor) {
            maxMotor = motor[i];
		}
	}
	
	for (i = 0; i < numberMotor; i++) {
        if (maxMotor > mcfg.maxthrottle && !f.FIXED_WING) {     // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - mcfg.maxthrottle;
		}
		
		if (feature(FEATURE_3D)) {
			if ((rcData[THROTTLE]) > mcfg.midrc) {
                motor[i] = constrain(motor[i], mcfg.deadband3d_high, mcfg.maxthrottle);
//				if ((mcfg.mixerConfiguration) == MULTITYPE_TRI) {
//                    servo[5] = constrain(servo[5], cfg.servoConf[5].min, cfg.servoConf[5].max);
//                }
			} else {
                motor[i] = constrain(motor[i], mcfg.mincommand, mcfg.deadband3d_low);
//                if ((mcfg.mixerConfiguration) == MULTITYPE_TRI) {
//                    servo[5] = constrain(servo[5], cfg.servoConf[5].max, cfg.servoConf[5].min);
//                }
			}
		} else {
			motor[i] = constrain(motor[i], mcfg.minthrottle, mcfg.maxthrottle);
			if ((rcData[THROTTLE]) < mcfg.mincheck) {
				if (!feature(FEATURE_MOTOR_STOP)) {
                    motor[i] = mcfg.minthrottle;
                } else {
                    motor[i] = mcfg.mincommand;
                    f.MOTORS_STOPPED = 1;
				}
			} else {
                f.MOTORS_STOPPED = 0;
            }
			
		}
		
		if (!f.ARMED) {
            motor[i] = motor_disarmed[i];
            f.MOTORS_STOPPED = 1;
        }
		
	}
}
