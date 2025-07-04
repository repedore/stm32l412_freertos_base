/*
 * task_aaaa.h
 *
 *  Created on: Jul 2, 2025
 *      Author: user
 */

#ifndef TASK_AAAA_H
#define TASK_AAAA_H

#include "FreeRTOS.h"
#include "queue.h"

#define TASK_AAAA_PRIORITY (tskIDLE_PRIORITY + 1)
#define TASK_AAAA_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)

void vTaskAAAA(void *pvParameters);

#endif /* TASK_AAAA_H */
