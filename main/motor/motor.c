#include "motor.h"
#include "esp_log.h"
#include "mpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_timer.h"

static const char *TAG = "MOTOR";

extern double tAngleX, tAngleY, tAngleZ;
extern uint32_t throttle;

// MPU 데이터 받아오는 구조체
extern mpu9250_data_t sensor_mpu9250_data;

// 내부 상태
static double AcXOff, AcYOff, AcZOff;
static double GyXOff, GyYOff, GyZOff;

static unsigned long t_prev = 0;

esp_err_t motor_init()
{
    const int motor_pins[] = {
        MOTOR_PIN_LU, MOTOR_PIN_RU, MOTOR_PIN_RL, MOTOR_PIN_LL
    };
    const ledc_channel_t motor_channels[] = {
        MOTOR_CHANNEL_LU, MOTOR_CHANNEL_RU, MOTOR_CHANNEL_RL, MOTOR_CHANNEL_LL
    };

    // set timer
    const ledc_timer_config_t ledc_timer = {
        .speed_mode = MOTOR_MODE,
        .duty_resolution = MOTOR_DUTY_RES,
        .timer_num = MOTOR_TIMER,
        .freq_hz = MOTOR_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&ledc_timer);

    if (err != ESP_OK)
        return err;

    // set channel
    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = MOTOR_MODE,
            .channel = motor_channels[i],
            .timer_sel = MOTOR_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = motor_pins[i],
            .duty = LEDC_DUTY_INIT,
            .hpoint = 0
        };

        err = ledc_channel_config(&ledc_channel);
        if (err != ESP_OK) return err;
    }

    stop_motor(); // stop은 반환값 없는 함수라고 가정
    return ESP_OK;
}

void accel_motor(const ledc_channel_t channel, const uint32_t speed)
{
    /* duty_percent: 0~100 (%) */
    // uint32_t calculated_duty = MOTOR_DUTY_MAX * duty_percent / 100;
    const uint32_t truncated_speed = speed <= MOTOR_DUTY_MAX ? speed : MOTOR_DUTY_MAX;

    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_MODE, channel, truncated_speed));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_MODE, channel));
}

void stop_motor()
{
    const ledc_channel_t motor_channels[] = {
        MOTOR_CHANNEL_LU, MOTOR_CHANNEL_RU, MOTOR_CHANNEL_RL, MOTOR_CHANNEL_LL
    };

    for (int i = 0; i < 4; i++)
    {
        accel_motor(motor_channels[i], (uint32_t) 0); // 듀티 0으로 설정
    }
}

void motor_task(void *pvParameters)
{
    static double ResX, ResY, ResZ;

    t_prev = esp_timer_get_time();
    mpu9250_data_t *p_mpu9250_data = get_mpu9250_data();

    while (1)
    {
        if (!sensor_mpu9250_data.is_calibrated)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }


        if (throttle == 0)
        {
            p_mpu9250_data->complemented_angle_x = 0;
            p_mpu9250_data->complemented_angle_y = 0;
            p_mpu9250_data->complemented_angle_z = 0;
        }

        // PID 제어
        const double eAngleX = tAngleX - p_mpu9250_data->complemented_angle_x;
        const double eAngleY = tAngleY - p_mpu9250_data->complemented_angle_y;
        const double eAngleZ = tAngleZ - p_mpu9250_data->complemented_angle_z;

        double BalX = Kp * eAngleX + Kd * -p_mpu9250_data->calidbrated_gyro_x;
        double BalY = Kp * eAngleY + Kd * -p_mpu9250_data->calidbrated_gyro_y;
        double BalZ = Kp * eAngleZ + Kd * -p_mpu9250_data->calidbrated_gyro_z;

        if (throttle == 0)
        {
            BalX = BalY = BalZ = 0.0;
        }

        ResX += Ki * eAngleX * p_mpu9250_data->delta_t;
        ResY += Ki * eAngleY * p_mpu9250_data->delta_t;
        ResZ += Ki * eAngleZ * p_mpu9250_data->delta_t;

        if (throttle == 0)
        {
            ResX = ResY = ResZ = 0.0;
        }

        BalX += ResX;
        BalY += ResY;
        BalZ += ResZ;

        // 모터 속도 계산
        const uint32_t speedA = throttle + BalX - BalY + BalZ;
        const uint32_t speedB = throttle - BalX - BalY - BalZ;
        const uint32_t speedC = throttle - BalX + BalY + BalZ;
        const uint32_t speedD = throttle + BalX + BalY - BalZ;

        // PWM 출력
        accel_motor(MOTOR_CHANNEL_LU, speedA);
        accel_motor(MOTOR_CHANNEL_RU, speedB);
        accel_motor(MOTOR_CHANNEL_RL, speedC);
        accel_motor(MOTOR_CHANNEL_LL, speedD);

        // 주기 10ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
