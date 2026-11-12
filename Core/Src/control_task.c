#include "control_task.h"
#include <stdint.h>
#include "tim.h"
#include "mpu6050.h"
#include "filters.h"
#include "DRV8874.h"
#include "pid.h"
#include "main.h"
#include "RC_Command.h"
#include "queue.h"
#include <math.h>


extern QueueHandle_t rcQueue;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void vControlTask(void *pvParameters){

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1);

	int16_t left_encoder_last_pos = 0;
	int16_t right_encoder_last_pos = 0;


	PID_Controller_t pid1 = {
			.kp = 10.f,
			.kd = 0.45f,
			.ki = 0.f,
			.dt = 0.001f,
			.error = 0.f,
			.integral = 0.f,
			.saturation = 100.f
	};

	PID_Controller_t pid_speed = {
			.kp = 1.5f,
			.ki = 0.1f,
			.kd = 0.f,
			.dt = 0.010f,
			.saturation = 5.f
	        };

	const float angle_setpoint_base = 0.f;
	float angle_setpoint = angle_setpoint_base;
	float wheel_speed_filt = 0.f;
	uint8_t speed_loop_cnt = 0;

	float speed_setpoint = 0.f;
	float turn = 0.f;
	float current_angle = 0.f;
	MPU6050_t scaled_data = {0};
	RC_Command_t command = {0};

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	DRV8874_init();

	vTaskDelay(pdMS_TO_TICKS(1000));
	mpu6050_init();
	vTaskDelay(pdMS_TO_TICKS(1000));



	xLastWakeTime = xTaskGetTickCount();
	for(;;){



		MPU6050_Raw_t raw_mpu_data;



		xQueueReceive(rcQueue, &command, 0);

		speed_setpoint = command.speed;
		turn = command.turn;

		if(mpu6050_read_all(&raw_mpu_data) == HAL_OK){
			mpu6050_convert(&raw_mpu_data, &scaled_data);
			current_angle = ComplementaryFilter(&scaled_data, 0.001);
			//printf("Current angle: %f\n", current_angle);
		}else{
			//printf("IMU failed\n");
		}

		if(current_angle > 50.f || current_angle < -50.f){
			stop_motors_fnc();
		}

		if(++speed_loop_cnt >= 10){
			speed_loop_cnt = 0;
            int16_t current_left  =  __HAL_TIM_GET_COUNTER(&htim3);
            int16_t current_right = -__HAL_TIM_GET_COUNTER(&htim4);

            int16_t d_left  = current_left  - left_encoder_last_pos;
            int16_t d_right = current_right - right_encoder_last_pos;

            left_encoder_last_pos  = current_left;
            right_encoder_last_pos = current_right;

            float wheel_speed = 0.5f * (d_left + d_right);
            wheel_speed_filt += 0.3f * (wheel_speed - wheel_speed_filt);

            angle_setpoint = angle_setpoint_base + pid_regulator(&pid_speed, speed_setpoint, wheel_speed_filt, 0.f);
		}

		//printf("Left: %d\nRight: %d\n", current_left, current_right);


		float angle_rate = -scaled_data.gyro_y;
		float pid_output = pid_regulator(&pid1, angle_setpoint, current_angle, angle_rate);

		if(fabsf(pid_output) + fabsf(turn) > 100){
			DRV8874_set_left_speed(pid_output);
			DRV8874_set_right_speed(pid_output);
		}else{
			DRV8874_set_left_speed(pid_output-turn);
			DRV8874_set_right_speed(pid_output+turn);
		}
		//printf("pid out: %d\n", pid_output);






		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
}
