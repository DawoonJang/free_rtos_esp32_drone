//
// Created by Dawoon Jang on 25. 4. 26.
//

#ifndef MPU9250_H
#define MPU9250_H

#include "esp_err.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 22         // I2C SCL 핀 번호
#define I2C_MASTER_SDA_IO 21         // I2C SDA 핀 번호
#define I2C_MASTER_NUM I2C_NUM_0     // I2C 포트 번호
#define I2C_MASTER_FREQ_HZ 400000    // I2C 클럭
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU9250_ADDR 0x68            // MPU9250 기본 I2C 주소
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_REG_ACCEL  0x3B

#define CALIBRATE_CNT  1000
#define RADIANS_TO_DEGREES (180.0f / 3.14159f)
#define GYROXYZ_TO_DEGREES_PER_SEC (131.0f)
#define ALPHA (0.96)

typedef struct
{
    int16_t acc_x,              acc_y,              acc_z;
    int16_t angular_velocity_x, angular_velocity_y, angular_velocity_z; // gyro sensor
    float   temp;

    int32_t acc_x_sum,              acc_y_sum,              acc_z_sum;
    int32_t angular_velocity_x_sum, angular_velocity_y_sum, angular_velocity_z_sum;

    double acc_x_offset,  acc_y_offset,  acc_z_offset;
    double angular_velocity_x_offset, angular_velocity_y_offset, angular_velocity_z_offset;

    bool    is_calibrated;
    int16_t cnt_calibrated;

    double complemented_angle_x, complemented_angle_y, complemented_angle_z;
    double calidbrated_angular_velocity_x,   calidbrated_angular_velocity_y,   calidbrated_angular_velocity_z;
    double delta_t;
} mpu9250_data_t;


esp_err_t init_i2c_master(void);

esp_err_t mpu9250_init(void);

esp_err_t mpu9250_read(void);

void mpu9250_task(void *pvParameters);

mpu9250_data_t *get_mpu9250_data(void);

#endif //MPU9250_H
