#include "DRV8874.h"
#include "tim.h"
#include "gpio.h"

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


void DRV8874_set_left_speed(float speed){

	if(stop_motors){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		return;
	}

	if (speed > 100.f)  speed = 100.f;
	else if(speed >=2.f && speed < 5.f) speed = 5.f;
	else if(speed > -2.f && speed < 2.f) speed = 0.f;
	else if(speed > -5.f && speed <= -2.f) speed = -5.f;
	if (speed < -100.f) speed = -100.f;

	if(speed > 100){
		HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 3999);
	}else if(speed > 0){
		HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)(speed * 39.99f));
	}else if(speed == 0){
		HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	}else if(speed > -100){
		HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)(-speed * 39.99f));
	}else{
		HAL_GPIO_WritePin(LEFT_MOTOR_PH_GPIO_Port, LEFT_MOTOR_PH_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 3999);
	}
}

void DRV8874_set_right_speed(float speed){

	if(stop_motors){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		return;
	}

	if (speed > 100.f)  speed = 100.f;
	else if(speed >=2.f && speed < 5.f) speed = 5.f;
	else if(speed > -2.f && speed < 2.f) speed = 0.f;
	else if(speed > -5.f && speed <= -2.f) speed = -5.f;
	if (speed < -100.f) speed = -100.f;

	if(speed >= 100){
		HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3999);
	}else if(speed > 0){
		HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)(speed * 39.99f));
	}else if(speed == 0){
		HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}else if(speed >= -100){
		HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)(-speed * 39.99f));
	}else{
		HAL_GPIO_WritePin(RIGHT_MOTOR_PH_GPIO_Port, RIGHT_MOTOR_PH_Pin, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3999);
	}
}

void stop_motors_fnc(void){
	stop_motors = 1;
	return;
}
