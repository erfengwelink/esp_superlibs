#include "app_gpios.h"
#include "global_cfg.h"
#include "string.h"
#include "esp_log.h"

const static char* TAG = "GPIO";
int app_gpios_init(void *param)
{
    ESP_LOGI(TAG, "%s\r\n", __func__);
    gpio_config_t io_conf;
    assert(upTable.gpio.gpioNum <= PRIPH_MAX_GPIOPIN_NUM);
    for(int i=0; i<upTable.gpio.gpioNum; i++)
    {
        memcpy(&io_conf,&upTable.gpio.gpiocfg[i], sizeof(upTable.gpio.gpiocfg[i]));
        gpio_config(&io_conf);
    }
    return 0;
}
