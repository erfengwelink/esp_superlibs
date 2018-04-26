#include "app_can.h"
#include "app_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "app_peripheral.h"
#include "app_console.h"

void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    app_can_setup();
    xTaskCreate(CAN_poll_task,"CAN_poll_task",2048, NULL, 5, NULL);
    //app_can_send(...); /* "..." depends on your applications.*/
    initialise_wifi();

    //peripheral_stuff_init();

    app_console_init();
    app_console_run();
}

