#ifndef _APP_CAN_H_
#define _APP_CAN_H_
#include "CAN.h"
#ifdef __cplusplus
extern "C" {
#endif

void app_can_setup();

void CAN_poll_task(void *ignore);

int app_can_send(const CAN_frame_t* p_frame);

int app_can_stop();

#ifdef __cplusplus
}
#endif

#endif