/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "motor/motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    static bool flag = false;
    init_motor();
    ESP_LOGI(TAG, "Motor initialization done.");

    while (1)
    {
        if (flag)
        {
            ESP_LOGI(TAG, "All motors set to 10%% duty");
            accel_motor(MOTOR_CHANNEL_RU, 20);
            accel_motor(MOTOR_CHANNEL_RL, 20);
            accel_motor(MOTOR_CHANNEL_LU, 20);
            accel_motor(MOTOR_CHANNEL_LL, 20);
        } else
        {
            ESP_LOGI(TAG, "Motors stopped");
            stop_motor(); // 멈추기
        }

        flag = !flag; // flag 토글
        vTaskDelay(pdMS_TO_TICKS(2000)); // 1초 대기
    }
}
