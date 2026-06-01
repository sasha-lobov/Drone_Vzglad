#include "bmp280_driver.h"
#include <math.h>

#define BMP280_ADDR      (0x76 << 1)
#define BMP280_REG_DATA  0xF7
#define BMP280_DATA_LEN  6

static I2C_HandleTypeDef *h_bmp_i2c;
static uint8_t bmp_raw[BMP280_DATA_LEN];
static volatile bool bmp_ready = false;

// Калибровка читается один раз при старте
typedef struct {
    uint16_t dig_T1, dig_P1;
    int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} BMP280_Cal_t;
static BMP280_Cal_t cal_data;

void BMP280_Init(I2C_HandleTypeDef *hi2c) {
    h_bmp_i2c = hi2c;
    uint8_t cal_buf[24];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT, cal_buf, 24, 100);
    cal_data.dig_T1 = (cal_buf[1] << 8) | cal_buf[0];
    cal_data.dig_T2 = (cal_buf[3] << 8) | cal_buf[2];
    cal_data.dig_T3 = (cal_buf[5] << 8) | cal_buf[4];
    cal_data.dig_P1 = (cal_buf[7] << 8) | cal_buf[6];
    cal_data.dig_P2 = (cal_buf[9] << 8) | cal_buf[8];
    cal_data.dig_P3 = (cal_buf[11] << 8) | cal_buf[10];
    cal_data.dig_P4 = (cal_buf[13] << 8) | cal_buf[12];
    cal_data.dig_P5 = (cal_buf[15] << 8) | cal_buf[14];
    cal_data.dig_P6 = (cal_buf[17] << 8) | cal_buf[16];
    cal_data.dig_P7 = (cal_buf[19] << 8) | cal_buf[18];
    cal_data.dig_P8 = (cal_buf[21] << 8) | cal_buf[20];
    cal_data.dig_P9 = (cal_buf[23] << 8) | cal_buf[22];
}

bool BMP280_StartAltReadDMA(void) {
    // Trigger forced measurement (OSR P=16, T=2, Mode=FORCED)
    uint8_t ctrl = 0xB7;
    if(HAL_I2C_Mem_Write(h_bmp_i2c, BMP280_ADDR, 0xF4, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 10) != HAL_OK)
        return false;
    HAL_Delay(10); // ожидание конверсии

    return (HAL_I2C_Mem_Read_DMA(h_bmp_i2c, BMP280_ADDR, BMP280_REG_DATA, I2C_MEMADD_SIZE_8BIT,
                                 bmp_raw, BMP280_DATA_LEN) == HAL_OK);
}

bool BMP280_GetAltitude(float *altitude) {
    if(!bmp_ready) return false;
    bmp_ready = false;

    int32_t adc_P = ((uint32_t)bmp_raw[0] << 12) | ((uint32_t)bmp_raw[1] << 4) | (bmp_raw[2] >> 4);
    // компенсация давления
    double var1, var2, p;
    var1 = ((double)adc_P/16384.0 - (double)cal_data.dig_P1/1024.0) * cal_data.dig_P2;
    var2 = (((double)adc_P/131072.0 - (double)cal_data.dig_P1/8192.0) *
            ((double)adc_P/131072.0 - (double)cal_data.dig_P1/8192.0)) * cal_data.dig_P3;
    p = ((double)adc_P/16384.0 - (double)cal_data.dig_P1/1024.0) * cal_data.dig_P2 + var2;

    double pressure_hPa = p / 256.0;
    *altitude = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 0.1903));
    return true;
}

void BMP280_DMA_Complete(void) { bmp_ready = true; }
