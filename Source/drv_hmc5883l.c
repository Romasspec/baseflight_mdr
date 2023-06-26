#include "board.h"

// HMC5883L, default address 0x1E

#define MAG_ADDRESS 0x0D//0x1E
#define MAG_DATA_REGISTER 0x03

static sensor_align_e magAlign = CW180_DEG;
static float magGain[3] = { 1.0f, 1.0f, 1.0f };

bool hmc5883lDetect(sensor_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(MAG_ADDRESS, 0x0A, 1, &sig);
    if (!ack || sig != 'H')
        return false;

    mag->init = hmc5883lInit;
    mag->read = hmc5883lRead;

    return true;
}

void hmc5883lInit(sensor_align_e align)
{
	if (align > 0)
        magAlign = align;
}

void hmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];
    int16_t mag[3];

    i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    mag[X] = (int16_t)(buf[0] << 8 | buf[1]) * magGain[X];
    mag[Z] = (int16_t)(buf[2] << 8 | buf[3]) * magGain[Z];
    mag[Y] = (int16_t)(buf[4] << 8 | buf[5]) * magGain[Y];
    alignSensors(mag, magData, magAlign);
}
