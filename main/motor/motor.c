#include "motor.h"

void init_motor() {
    const int motor_pins[] = {
        MOTOR_PIN_LU, MOTOR_PIN_RU, MOTOR_PIN_RL, MOTOR_PIN_LL
    };
    const ledc_channel_t motor_channels[] = {
        MOTOR_CHANNEL_LU, MOTOR_CHANNEL_RU, MOTOR_CHANNEL_RL, MOTOR_CHANNEL_LL
    };

    // set timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = MOTOR_MODE,
        .duty_resolution  = MOTOR_DUTY_RES,
        .timer_num        = MOTOR_TIMER,
        .freq_hz          = MOTOR_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // set channel
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = MOTOR_MODE,
            .channel        = motor_channels[i],
            .timer_sel      = MOTOR_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motor_pins[i],
            .duty           = LEDC_DUTY_INIT,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    stop_motor();
}

void accel_motor(const ledc_channel_t channel, const uint8_t duty_percent) {
    /* duty_percent: 0~100 (%) */
    uint32_t calculated_duty =  MOTOR_DUTY_MAX * duty_percent / 100;

    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_MODE, channel, calculated_duty));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_MODE, channel));
}

void stop_motor() {
    const ledc_channel_t motor_channels[] = {
        MOTOR_CHANNEL_LU, MOTOR_CHANNEL_RU, MOTOR_CHANNEL_RL, MOTOR_CHANNEL_LL
    };

    for (int i = 0; i < 4; i++) {
        accel_motor(motor_channels[i], 0); // 듀티 0으로 설정
    }
}
