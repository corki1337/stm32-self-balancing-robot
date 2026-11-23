#pragma once
#include <stdint.h>

typedef enum{
	MOTOR_LEFT,
	MOTOR_RIGHT
}MotorSide_t;


void DRV8874_init(void);

void DRV8874_set_speed(MotorSide_t side, float speed);

void stop_motors_fnc(void);
