#pragma once

#include <stdint.h>


typedef struct{
	uint32_t tick;
	float complementary_angle;
	float kalman_angle;
	float angle_sp;
	float pid_output;
	float wheel_speed;
}Log_Frame_t;
