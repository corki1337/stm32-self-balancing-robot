#include "encoder.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


void encoder_start(void){
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

int16_t encoder_read_left(void){
	return __HAL_TIM_GET_COUNTER(&htim3);
}

int16_t encoder_read_right(void){
	return -__HAL_TIM_GET_COUNTER(&htim4);
}
