#include "motor_logic.h"
#include <math.h>


#define DEAD_ZONE 2.f
#define MINIMAL_SPEED 5.f
#define MAX_SPEED 100.f
#define MOTOR_PWM_MAX 3999.f

int16_t motor_logic(float speed){

	if(isnan(speed)) return 0;

	if (speed > MAX_SPEED)  speed = MAX_SPEED;
	else if(speed >=DEAD_ZONE && speed < MINIMAL_SPEED) speed = MINIMAL_SPEED;
	else if(speed > -DEAD_ZONE && speed < DEAD_ZONE) speed = 0.f;
	else if(speed > -MINIMAL_SPEED && speed <= -DEAD_ZONE) speed = -MINIMAL_SPEED;
	else if (speed < -MAX_SPEED) speed = -MAX_SPEED;


	return (int16_t)(speed / MAX_SPEED * MOTOR_PWM_MAX);
}
