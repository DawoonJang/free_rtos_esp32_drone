//
// Created by Dawoon Jang on 25. 4. 26.
//

#include "mpu.h"
#include "esp_log.h"

static const char *TAG = "MPU";

esp_err_t init_i2c_master(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t mpu9250_init(void)
{
    uint8_t data[2];
    data[0] = MPU9250_PWR_MGMT_1;
    data[1] = 0x00; // Sleep 모드 해제

    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU9250_ADDR,
        data,
        sizeof(data),
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "MPU9250 initialized successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "MPU9250 init failed");
    }

    return ret;
}

esp_err_t mpu9250_read(mpu9250_data_t *data)
{
    uint8_t raw_data[14];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU9250_REG_ACCEL, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 13, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &raw_data[13], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
        return ret;

    data->acc_x = (raw_data[0] << 8) | raw_data[1];
    data->acc_y = (raw_data[2] << 8) | raw_data[3];
    data->acc_z = (raw_data[4] << 8) | raw_data[5];
    data->temp = (raw_data[6] << 8) | raw_data[7];
    data->gyro_x = (raw_data[8] << 8) | raw_data[9];
    data->gyro_y = (raw_data[10] << 8) | raw_data[11];
    data->gyro_z = (raw_data[12] << 8) | raw_data[13];
    data->temp = (data->temp / 340.0f) + 36.53f;

    return ESP_OK;
}
