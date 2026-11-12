#pragma once
#include <stdint.h>
#include "stm32l4xx_hal.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_Raw_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}MPU6050_t;


void mpu6050_init(void);

HAL_StatusTypeDef mpu6050_read_all(MPU6050_Raw_t *data);

void mpu6050_convert(const MPU6050_Raw_t *raw_data, MPU6050_t *data);
