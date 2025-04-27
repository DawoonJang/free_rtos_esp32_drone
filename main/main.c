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
    initialize();

    xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);
    xTaskCreate(mpu9250_task, "sensor_task", 4096, NULL, 5, NULL);
}
