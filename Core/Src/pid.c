#include "pid.h"



float pid_regulator(PID_Controller_t *pid, float setpoint, float measurement, float measurement_rate){

	pid->error = setpoint-measurement;

	float proportional = pid->error * pid->kp;
	float derivative = -pid->kd * measurement_rate;

	float current_integral = pid->error * pid->dt;
	pid->integral += current_integral;
	float integral = pid->ki * pid->integral;


	float output = proportional + derivative +integral;

	if(output > pid->saturation){
		output = pid->saturation;
		pid->integral -= current_integral;
	}else if(output < -pid->saturation){
		output = -pid->saturation;
		pid->integral -= current_integral;
	}

	return output;
}
