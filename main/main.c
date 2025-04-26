#include "driver/ledc.h"
#include "esp_err.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor/motor.h"
#include "mpu/mpu.h"

static const char *TAG = "MAIN";

static void initialize(void)
{
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(init_i2c_master());
    ESP_ERROR_CHECK(mpu9250_init());
}

void app_main(void)
{
    static bool flag;

    initialize();

    while (1)
    {
        if (flag)
        {
            mpu9250_data_t sensor_mpu9250_data = {0};
            ESP_LOGI(TAG, "All motors set to 10%% duty");
            accel_motor(MOTOR_CHANNEL_RU, 20);
            accel_motor(MOTOR_CHANNEL_RL, 20);
            accel_motor(MOTOR_CHANNEL_LU, 20);
            accel_motor(MOTOR_CHANNEL_LL, 20);

            ESP_ERROR_CHECK(mpu9250_read(&sensor_mpu9250_data));
            ESP_LOGI(TAG, "AccX: %d, AccY: %d, AccZ: %d", sensor_mpu9250_data.acc_x, sensor_mpu9250_data.acc_y,
                     sensor_mpu9250_data.acc_z);
            ESP_LOGI(TAG, "Gyro X:%d Y:%d Z:%d", sensor_mpu9250_data.gyro_x, sensor_mpu9250_data.gyro_y,
                     sensor_mpu9250_data.gyro_z);
        }
        else
        {
            ESP_LOGI(TAG, "Motors stopped");
            stop_motor();
        }

        flag = !flag; // flag 토글
        vTaskDelay(pdMS_TO_TICKS(2000)); // 1초 대기
    }
}
