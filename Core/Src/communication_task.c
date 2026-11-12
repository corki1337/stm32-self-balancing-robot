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

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer, 64);

	for(;;){
		uint32_t curr_size = 0;
		BaseType_t received = pdFALSE;
		received = xTaskNotifyWait(0, UINT32_MAX, &curr_size, pdMS_TO_TICKS(200));
		RC_Command_t command = {0};

		if(received == pdTRUE){

			if(decode_rc_frame(buffer, curr_size, &channels)){


				float turn = (channels.channel_01 < 992 + DEAD_ZONE && channels.channel_01 > 992 - DEAD_ZONE) ? 0 : (float)(channels.channel_01 - 992) * (MAX_TURN/819.f);
				float speed = (channels.channel_02 < 992 + DEAD_ZONE && channels.channel_02 > 992 - DEAD_ZONE) ? 0 : (float)(channels.channel_02 - 992) * (MAX_SPEED/819.f);

				command.speed = speed;
				command.turn = turn;

				xQueueOverwrite(rcQueue, &command);
			}
		}else{
			xQueueOverwrite(rcQueue, &command);
		}

		HAL_UART_AbortReceive(&huart1);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buffer, 64);
	}

}
