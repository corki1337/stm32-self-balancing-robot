#include "DRV8874.h"
#include "tim.h"
#include "gpio.h"

#include "motor_logic.h"

extern TIM_HandleTypeDef htim2;

static uint8_t stop_motors = 0;

void DRV8874_init(void){

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 0);


}


void DRV8874_set_speed(MotorSide_t side, float speed){
	if(stop_motors){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		return;
	}

	int16_t pwm_speed = motor_logic(speed);

	if(side == MOTOR_LEFT){
		if(pwm_speed > 0) HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 1);
		else HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 0);

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (pwm_speed > 0) ? pwm_speed : -pwm_speed);
	}else{
		if(pwm_speed > 0)HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 1);
		else HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 0);

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (pwm_speed > 0) ? pwm_speed : -pwm_speed);
	}
}




void stop_motors_fnc(void){
	stop_motors = 1;
	return;
}
