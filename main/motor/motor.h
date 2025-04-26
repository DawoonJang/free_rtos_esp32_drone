//
// Created by Dawoon Jang on 25. 4. 24.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "esp_err.h"

#define LED 2;
#define MOTOR_PIN_LU 23 // Left Upper
#define MOTOR_PIN_RU 19 // Right Upper
#define MOTOR_PIN_RL 18 // Right Lower
#define MOTOR_PIN_LL 26 // Left Lower

#define MOTOR_CHANNEL_LU LEDC_CHANNEL_1
#define MOTOR_CHANNEL_RU LEDC_CHANNEL_2
#define MOTOR_CHANNEL_RL LEDC_CHANNEL_3
#define MOTOR_CHANNEL_LL LEDC_CHANNEL_4

#define MOTOR_TIMER         LEDC_TIMER_0
#define MOTOR_MODE          LEDC_HIGH_SPEED_MODE
#define MOTOR_DUTY_RES      LEDC_TIMER_10_BIT
#define MOTOR_DUTY_MAX      ((1 << MOTOR_DUTY_RES) - 1)
#define MOTOR_FREQ          5000
#define LEDC_DUTY_INIT      0

esp_err_t motor_init();

void accel_motor(ledc_channel_t channel, uint8_t duty_percent);

void stop_motor();

#endif //MOTOR_H
