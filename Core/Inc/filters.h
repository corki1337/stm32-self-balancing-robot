#pragma once

#include "imu_data.h"



float ComplementaryFilter(const IMU_t *data, float dt);
