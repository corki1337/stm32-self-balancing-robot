#include "logging_task.h"
#include "usart.h"
#include "main.h"


#include "queue.h"
#include "Log_Frame.h"
#include <stdio.h>

extern QueueHandle_t logQueue;

extern UART_HandleTypeDef huart2;

void vLoggingTask(void *pvParameters){


	Log_Frame_t frame = {0};
	char buf[96];

	for(;;){

		if(xQueueReceive(logQueue, &frame, portMAX_DELAY) == pdPASS){

			int len = snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f,%.2f,%.2f\n\r",frame.tick,frame.complementary_angle,frame.kalman_angle,frame.angle_sp,frame.pid_output,frame.wheel_speed);

			if(len>0){
				HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 10);
			}
		}
	}
}
