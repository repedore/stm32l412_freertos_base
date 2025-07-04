/* 큐가 보유할 수 있는 항목 수입니다.
수신 태스크가 송신 태스크보다 우선순위가 높기 때문에 항목이 추가될 때 즉시 제거하므로, 송신 태스크는 항상 큐가 비어 있어야 합니다. */

#include "cmsis_os2.h"

#define mainQUEUE_LENGTH (1)

#define msg1QUEUE_LENGTH (10)
#define msg2QUEUE_LENGTH (10)
typedef struct
{
    uint8_t button_id; /* 1: 버튼 1, 2: 버튼 2 */
    uint8_t state;     /* 1: 눌림, 0: 릴리스 */
} ButtonEvent_t;

extern osMessageQueueId_t xMsg1Queue;
extern osMessageQueueId_t xMsg2Queue;
extern osMessageQueueId_t xMsg3Queue;