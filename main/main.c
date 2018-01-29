#include "app_can.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void app_main()
{
    app_can_setup();
    xTaskCreate(CAN_poll_task,"CAN_poll_task",2048, NULL, 5, NULL);
    //app_can_send(...); /* "..." depends on your applications.*/

}

