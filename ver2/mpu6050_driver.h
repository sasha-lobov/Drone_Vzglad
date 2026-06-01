#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "main.h"
#include <stdbool.h>

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} MPU6050_Data_t;

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
bool MPU6050_StartDMA(void);
bool MPU6050_GetData(MPU6050_Data_t *data);

#endif
