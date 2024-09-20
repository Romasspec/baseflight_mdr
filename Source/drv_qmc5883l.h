#pragma once

bool qmc5883lDetect(sensor_t *mag);
void qmc5883lInit(sensor_align_e align);
void qmc5883lRead(int16_t *magData);

