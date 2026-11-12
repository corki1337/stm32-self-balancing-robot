#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t rcQueue;


void vCommunicationTask(void *pvParameters);

