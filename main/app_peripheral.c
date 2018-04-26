#include "app_peripheral.h"
//#include "app_gpios.h"
#include <stddef.h>
#include "global_cfg.h"

int peripheral_stuff_init()
{
    // gpios init
    if (upTable.gpio.name == PRIPH_GPIO)
    {
        upTable.gpio.init(NULL);
    }

    //  i2c init

    if (upTable.i2c.name == PRIPH_I2C)
    {
        upTable.i2c.init(NULL);
    }

    return 0;
}

int peripheral_stuff_deinit()
{

    return 0;
}