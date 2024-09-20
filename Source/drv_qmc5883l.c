#include "board.h"

// QMC5883L, default address 0x0D

#define QMC5883L_MAG_ADDRESS 0x0D
#define MAG_DATA_REGISTER 0x00

#define QMC58X3_R_CONTROL1 0x09
	#define OSR_POS							6
	#define OSR_512							(0<<OSR_POS)
	#define OSR_256							(1<<OSR_POS)
	#define OSR_128							(2<<OSR_POS)
	#define OSR_64							(3<<OSR_POS)

	#define RNG_POS							4
	#define RNG_2G							(0<<RNG_POS)
	#define RNG_8G							(1<<RNG_POS)

	#define ODR_POS							2
	#define ODR_10							(0<<ODR_POS)
	#define ODR_50							(1<<ODR_POS)
	#define ODR_100							(2<<ODR_POS)
	#define ODR_200							(3<<ODR_POS)

	#define MODE_POS						0
	#define MODE_STANDBY				(0<<MODE_POS)
	#define MODE_CONTINUOUS			(1<<MODE_POS)

#define QMC58X3_R_CONTROL2 0x0A
	#define SOFT_RST_POS				7
	#define SOFT_RST						(1<<SOFT_RST_POS)

	#define ROL_PNT_POS					6
	#define ROL_PNT							(1<<ROL_PNT_POS)
	
	#define INT_ENB_POS					0
	#define INT_ENB							(0<<INT_ENB_POS)
	#define INT_DSB							(1<<INT_ENB_POS)
	
#define		SET_RESET_PR 0x0B

//#define QMC58X3_R_MODE 2
#define QMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define QMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define QMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.

//#define QMC_POS_BIAS 1
//#define QMC_NEG_BIAS 2

static sensor_align_e magAlign = CW180_DEG;
static float magGain[3] = { 1.0f, 1.0f, 1.0f };

bool qmc5883lDetect(sensor_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(QMC5883L_MAG_ADDRESS, 0x0D, 1, &sig);
    if (!ack || sig != 0xFF)
        return false;

    mag->init = qmc5883lInit;
    mag->read = qmc5883lRead;

    return true;
}

void qmc5883lInit(sensor_align_e align)
{
	int16_t magADC[3];
	int i;
	int32_t xyz_total[3] = { 0, 0, 0 };		// 32 bit totals so they won't overflow.
	bool bret = true;						// Error indicator

	if (align > 0)
        magAlign = align;
	
	delay(50);
    i2cWrite(QMC5883L_MAG_ADDRESS, QMC58X3_R_CONTROL1, OSR_256|RNG_2G|ODR_10|MODE_CONTINUOUS);   // Reg A DOR = 0x010 + MS1, MS0 set to pos bias
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    i2cWrite(QMC5883L_MAG_ADDRESS, QMC58X3_R_CONTROL2, ROL_PNT|INT_DSB); // Set the Gain to 2.5Ga (7:5->011)
    i2cWrite(QMC5883L_MAG_ADDRESS, SET_RESET_PR, 0x01);
		delay(100);
    qmc5883lRead(magADC);
	
	for(i = 0; i < 10; i++) { // Collect 10 samples
		
        delay(50);
        qmc5883lRead(magADC);       // Get the raw values in case the scales have already been changed.
		
		// Since the measurements are noisy, they should be averaged rather than taking the max.
        xyz_total[X] += magADC[X];
        xyz_total[Y] += magADC[Y];
        xyz_total[Z] += magADC[Z];
		
		// Detect saturation.
        if (-4096 >= min(magADC[X], min(magADC[Y], magADC[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        LED1_TOGGLE;
	}
	
	// Apply the negative bias. (Same gain)
    i2cWrite(QMC5883L_MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
	
	for (i = 0; i < 10; i++) {
        i2cWrite(QMC5883L_MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(50);
        hmc5883lRead(magADC);               // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged.
        xyz_total[X] -= magADC[X];
        xyz_total[Y] -= magADC[Y];
        xyz_total[Z] -= magADC[Z];

        // Detect saturation.
        if (-4096 >= min(magADC[X], min(magADC[Y], magADC[Z]))) {
            bret = false;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
        LED1_TOGGLE;
    }
	
		magGain[X] = fabsf(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
    magGain[Y] = fabsf(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
    magGain[Z] = fabsf(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);
	
	 // leave test mode
    i2cWrite(QMC5883L_MAG_ADDRESS, HMC58X3_R_CONFA, 0x70);   // Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2cWrite(QMC5883L_MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2cWrite(QMC5883L_MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);

    if (!bret) {                // Something went wrong so get a best guess
        magGain[X] = 1.0f;
        magGain[Y] = 1.0f;
        magGain[Z] = 1.0f;
    }	
}

void qmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];
    int16_t mag[3];

    i2cRead(QMC5883L_MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    mag[X] = (int16_t)(buf[1] << 8 | buf[0]) * magGain[X];
    mag[Z] = (int16_t)(buf[3] << 8 | buf[2]) * magGain[Z];
    mag[Y] = (int16_t)(buf[5] << 8 | buf[4]) * magGain[Y];
    alignSensors(mag, magData, magAlign);
}
