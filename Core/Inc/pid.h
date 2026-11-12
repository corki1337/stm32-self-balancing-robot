#pragma once


typedef struct{
	float kp;
	float kd;
	float ki;
	float dt;
	float error;
	float integral;
	float saturation;
} PID_Controller_t;



float pid_regulator(PID_Controller_t *pid, float setpoint, float measurement, float measurement_rate);
