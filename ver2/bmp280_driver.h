#ifndef BMP280_DRIVER_H
#define BMP280_DRIVER_H

#include "main.h"
#include <stdbool.h>

void BMP280_Init(I2C_HandleTypeDef *hi2c);
bool BMP280_StartAltReadDMA(void);
bool BMP280_GetAltitude(float *altitude);

#endif
