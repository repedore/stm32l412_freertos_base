/*
 * task_aaaa.c
 *
 *  생성일: 2025년 7월 2일 작성자: user
 */

#include "task_aaaa.h"
#include "system_config.h"
//#include "hardware.h"

void vTaskAAAA(void *pvParameters) {
    for(;;){
        
    }
   ButtonEvent_t event;
   for(;;) {
       if(xQueueReceive(xMsg1Queue, &event, portMAX_DELAY) == pdPASS) {
           if(event.button_id == 1) {
               xQueueSend(xMsg2Queue, &event, portMAX_DELAY);
           } else if(event.button_id == 2) {
               xQueueSend(xMsg3Queue, &event, portMAX_DELAY);
           }
       }
   }
}

