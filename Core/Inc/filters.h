#pragma once

#include "mpu6050.h"



float ComplementaryFilter(const MPU6050_t *data, float dt);
