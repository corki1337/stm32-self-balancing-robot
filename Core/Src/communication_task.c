#include "communication_task.h"
#include "usart.h"
#include "crsf.h"
#include "RC_Command.h"

#include "main.h"

#define DEAD_ZONE 8
#define MAX_SPEED 5.f
#define MAX_TURN 30.f

static uint8_t buffer[64] = {0};
static TaskHandle_t xTaskToNotify = NULL;

static float scale_channel(uint16_t raw, float max_out){
	int32_t delta = (int32_t)raw - CRSF_CHANNEL_MID;
	if(delta > -DEAD_ZONE && delta < DEAD_ZONE) return 0.f;
	return (float)delta * (max_out / CRSF_CHANNEL_HALF_RANGE);
}

static void start_rx(void){
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer, sizeof(buffer));
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if(huart == &huart1){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(xTaskToNotify, Size, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	}
}



void vCommunicationTask(void *pvParameters){


	RC_Channels_t channels = {0};

	xTaskToNotify = xTaskGetCurrentTaskHandle();

	huart1.ErrorCode = HAL_UART_ERROR_NONE;
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	__HAL_UART_CLEAR_NEFLAG(&huart1);
	__HAL_UART_CLEAR_FEFLAG(&huart1);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);

	start_rx();

	for(;;){
		uint32_t curr_size = 0;
		BaseType_t received = pdFALSE;
		received = xTaskNotifyWait(0, UINT32_MAX, &curr_size, pdMS_TO_TICKS(200));
		RC_Command_t command = {0};

		if(received == pdTRUE){

			if(decode_rc_frame(buffer, curr_size, &channels)){


				command.turn  = scale_channel(channels.ch[0], MAX_TURN);
				command.speed = scale_channel(channels.ch[1], MAX_SPEED);

				xQueueOverwrite(rcQueue, &command);
			}
		}else{
			xQueueOverwrite(rcQueue, &command);
		}

		HAL_UART_AbortReceive(&huart1);
		start_rx();
	}

}
