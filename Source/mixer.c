#include "board.h"
#include "mw.h"

static uint8_t numberRules = 0;
int16_t motor[MAX_MOTORS];
int16_t motor_disarmed[MAX_MOTORS];
int16_t servo[MAX_SERVOS] = { 1500, 1500, 1500, 1500};
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
  
	// enable servos for mixes that require them. note, this shifts motor counts.
	core.useServo = mixers[mcfg.mixerConfiguration].useServo;
									
}

void writeMotors(void)
{
//    uint8_t i;

//    for (i = 0; i < numberMotor; i++)
//        pwmWriteMotor(i, motor[i]);
}

void writeAllMotors(int16_t mc)
{
//    uint8_t i;

//    // Sends commands to all motors
//    for (i = 0; i < numberMotor; i++)
//        motor[i] = mc;
    writeMotors();
}
