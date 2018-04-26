#ifndef _G_CFG_H_
#define _G_CFG_H_

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "app_gpios.h"
#include "app_i2cs.h"

#if 0
typedef struct {
    uint64_t pin_bit_mask;          /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_mode_t mode;               /*!< GPIO mode: set input/output mode                     */
    gpio_pullup_t pull_up_en;       /*!< GPIO pull-up                                         */
    gpio_pulldown_t pull_down_en;   /*!< GPIO pull-down                                       */
    gpio_int_type_t intr_type;      /*!< GPIO interrupt type                                  */
} gpio_config_t;


typedef struct gpio_entry{
    priph_t name;
    int gpioNum;
    gpio_config_t cfg[PRIPH_MAX_GPIOPIN_NUM];
    init_func_p init;
}gpio_entry;

typedef struct i2c_entry{
    priph_t name;
    int i2cNum;
    i2c_config_t cfg[PRIPH_MAX_I2C_NUM];
    init_func_p init;
}i2c_entry;

typedef struct usr_prip_table_t
{
    gpio_entry gpio;
    i2c_entry i2c;
} usr_prip_table_t;

#endif

#define PRIPH_MAX_GPIOPIN_NUM 6
#define PRIPH_MAX_I2C_NUM 1
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ    200000
typedef enum{
    PRIPH_GPIO = 1,
    PRIPH_I2C,
    PRIPH_I2S,
    PRIPH_ADC,
    PRIPH_UART,
    PRIPH_BLE,
}priph_t;

typedef int (*init_func_p)(void *param);

#if 0
typedef struct {
    uint64_t pin_bit_mask;          /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_mode_t mode;               /*!< GPIO mode: set input/output mode                     */
    gpio_pullup_t pull_up_en;       /*!< GPIO pull-up                                         */
    gpio_pulldown_t pull_down_en;   /*!< GPIO pull-down                                       */
    gpio_int_type_t intr_type;      /*!< GPIO interrupt type                                  */
} gpio_config_t;

typedef struct {
    uini8_t scl_pin;
    uini8_t sda_pin;
}i2c_config_t;
#endif

#if 0
typedef struct{
    i2c_mode_t mode;       /*!< I2C mode */
    gpio_num_t sda_io_num;        /*!< GPIO number for I2C sda signal */
    gpio_pullup_t sda_pullup_en;  /*!< Internal GPIO pull mode for I2C sda signal*/
    gpio_num_t scl_io_num;        /*!< GPIO number for I2C scl signal */
    gpio_pullup_t scl_pullup_en;  /*!< Internal GPIO pull mode for I2C scl signal*/

    union {
        struct {
            uint32_t clk_speed;     /*!< I2C clock frequency for master mode, (no higher than 1MHz for now) */
        } master;
        struct {
            uint8_t addr_10bit_en;  /*!< I2C 10bit address mode enable for slave mode */
            uint16_t slave_addr;    /*!< I2C address for slave mode */
        } slave;

    };
}i2c_config_t;
#endif
typedef struct{
    int mode;
    int sdaPin;
    int sclPin;
    int speed;
    uint8_t sdaPullup;
    uint8_t sclPullup;

}i2cs_config_t;

typedef struct gpio_entry{
    priph_t name;
    int gpioNum;
    gpio_config_t gpiocfg[PRIPH_MAX_GPIOPIN_NUM];
    init_func_p init;
}gpio_entry;

typedef struct i2c_entry{
    priph_t name;
    int i2cNum;
    i2cs_config_t i2ccfg[PRIPH_MAX_I2C_NUM];
    init_func_p init;
}i2c_entry;

typedef struct i2s_entry{
    int gpioNum;
    //gpio_config_t cfg[PRIPH_MAX_PIN_NUM];
}i2s_entry;

typedef struct adc_entry{
    int gpioNum;
    //gpio_config_t cfg[PRIPH_MAX_PIN_NUM];
}adc_entry;

typedef struct uart_entry{
    int gpioNum;
    //gpio_config_t cfg[PRIPH_MAX_PIN_NUM];
}uart_entry;


typedef struct usr_prip_table_t
{
    gpio_entry gpio;
    i2c_entry i2c;
} usr_prip_table_t;

static usr_prip_table_t upTable = {
    // for example : gpio2 out | gpio4 in 
    {PRIPH_GPIO, // pripherial type
    2, // valid entry num
        // fact entries:
        {   {GPIO_SEL_2, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},
            {GPIO_SEL_4, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE},   
            {0},
            {0},
            {0},
            {0}
        },
        app_gpios_init  //pripherial entry init function pointer
    },

#if 0
typedef struct{
    int mode;
    int sdaPin;
    int sclPin;
    uint8_t sdaPullup;
    uint8_t sclPullup;
}i2cs_config_t;
#endif

    {PRIPH_I2C, 
    1,
        {
            { I2C_MODE_MASTER,  GPIO_NUM_14, GPIO_NUM_13, I2C_MASTER_FREQ_HZ, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE},
        },
        app_i2cs_init
    },
        
    
    
/*
    {PRIPH_I2C, {0, {0}}, NULL},
    {PRIPH_I2S, {0, {0}}, NULL},
*/

};


#endif