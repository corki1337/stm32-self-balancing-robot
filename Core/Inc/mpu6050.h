#pragma once
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "imu_data.h"
#include <stdbool.h>



void mpu6050_init(void);

bool mpu6050_read_all(IMU_Raw_t *data);

void mpu6050_convert(const IMU_Raw_t *raw_data, IMU_t *data);
