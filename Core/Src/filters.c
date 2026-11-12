#include "filters.h"
#include "math.h"

#define ALPHA 0.998f
#define RAD_TO_DEG 57.2957795f


float ComplementaryFilter(const MPU6050_t *data, float dt){

	static float prev_angle = 0;

	float current_angle = ALPHA * (prev_angle - (data->gyro_y * dt)) + (1-ALPHA) *
			(atan2f(data->accel_x, sqrtf((data->accel_y * data->accel_y) + (data->accel_z * data->accel_z)))
					* RAD_TO_DEG);

	prev_angle = current_angle;

	return current_angle;

}
