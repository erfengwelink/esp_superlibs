#include "app_i2cs.h"
#include "global_cfg.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char* TAG = "I2c";


#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */

#define WRITE_BIT      I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT       I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0     /*!< I2C ack value */
#define NACK_VAL       0x1     /*!< I2C nack value */
#define I2C_MAX_TIMEOUT_MS   (500 / portTICK_RATE_MS)

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t addr, uint8_t ctrl, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ctrl, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MAX_TIMEOUT_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t addr, uint8_t ctrl, uint8_t* data_rd, size_t size)
{
    esp_err_t ret;
    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, ctrl, ACK_CHECK_EN);

    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | READ_BIT, ACK_CHECK_EN);
	if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MAX_TIMEOUT_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

int app_i2cs_init(void *param)
{
    i2c_config_t conf;
    ESP_LOGI(TAG, "%s\r\n", __func__);
    assert(upTable.i2c.i2cNum <= PRIPH_MAX_I2C_NUM);
    for(int i=0; i<upTable.i2c.i2cNum; i++)
    {
        conf.mode = upTable.i2c.i2ccfg[i].mode;
        conf.sda_io_num = upTable.i2c.i2ccfg[i].sdaPin;
        conf.sda_pullup_en = upTable.i2c.i2ccfg[i].sdaPullup;
        conf.scl_io_num = upTable.i2c.i2ccfg[i].sclPin;
        conf.scl_pullup_en = upTable.i2c.i2ccfg[i].sclPullup;
        conf.master.clk_speed = upTable.i2c.i2ccfg[i].speed;

        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    }
    return 0;
}