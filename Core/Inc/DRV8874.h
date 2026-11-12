#pragma once
#include <stdint.h>

void DRV8874_init(void);


void DRV8874_set_left_speed(float speed);

void DRV8874_set_right_speed(float speed);

void stop_motors_fnc(void);
