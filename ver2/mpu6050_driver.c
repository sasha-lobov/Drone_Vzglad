#include <mpu6050_driver.h>
#include <string.h>

#define MPU6050_ADDR        (0x68 << 1)
#define MPU6050_REG_START   0x3B
#define MPU6050_DATA_LEN    14
#define MPU6050_DMA_TIMEOUT 10

static I2C_HandleTypeDef *h_mpu_i2c;
static volatile uint8_t mpu_raw[MPU6050_DATA_LEN];
static volatile bool mpu_data_ready = false;
static uint8_t dma_retry = 0;

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    h_mpu_i2c = hi2c;
    // Разблокировка I2C (F1 workaround)
    if(HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
        __HAL_I2C_DISABLE(hi2c);
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BUSY);
        __HAL_I2C_ENABLE(hi2c);
    }
    MPU6050_StartDMA();
}

bool MPU6050_StartDMA(void) {
    if(HAL_I2C_Mem_Read_DMA(h_mpu_i2c, MPU6050_ADDR, MPU6050_REG_START, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)mpu_raw, MPU6050_DATA_LEN) == HAL_OK) {
        dma_retry = 0;
        return true;
    }
    return false;
}

bool MPU6050_GetData(MPU6050_Data_t *data) {
    if(!mpu_data_ready) return false;
    
    __disable_irq();
    mpu_data_ready = false;
    __enable_irq();
    
    data->ax = (int16_t)((mpu_raw[0] << 8) | mpu_raw[1]);
    data->ay = (int16_t)((mpu_raw[2] << 8) | mpu_raw[3]);
    data->az = (int16_t)((mpu_raw[4] << 8) | mpu_raw[5]);
    data->temp = (int16_t)((mpu_raw[6] << 8) | mpu_raw[7]);
    data->gx = (int16_t)((mpu_raw[8] << 8) | mpu_raw[9]);
    data->gy = (int16_t)((mpu_raw[10] << 8) | mpu_raw[11]);
    data->gz = (int16_t)((mpu_raw[12] << 8) | mpu_raw[13]);
    return true;
}

// Вызывается из HAL_I2C_MemRxCpltCallback в stm32f1xx_it.c
void MPU6050_DMA_Complete(void) {
    mpu_data_ready = true;
    // Перезапуск DMA в следующем цикле
}

// Обработка ошибок I2C
void MPU6050_Error_Recovery(void) {
    if(dma_retry++ > 3) {
        // Hard reset шины
        __HAL_I2C_DISABLE(h_mpu_i2c);
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        MX_I2C1_Init();
        dma_retry = 0;
    }
    HAL_I2C_DMAStop(h_mpu_i2c);
    __HAL_I2C_CLEAR_FLAG(h_mpu_i2c, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_ARLO);
    MPU6050_StartDMA();
}