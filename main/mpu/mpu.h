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
    // --- 센서값 읽어서 자세각 계산 ---
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float temp;

    int32_t acc_x_sum, acc_y_sum, acc_z_sum;
    int32_t gyro_x_sum, gyro_y_sum, gyro_z_sum;

    double acc_x_offset, acc_y_offset, acc_z_offset;
    double gyro_x_offset, gyro_y_offset, gyro_z_offset;

    bool is_calibrated;
    int16_t cnt_calibrated;

    double complemented_angle_x, complemented_angle_y, complemented_angle_z;
    double calidbrated_gyro_x, calidbrated_gyro_y, calidbrated_gyro_z;
    double delta_t;
} mpu9250_data_t;


esp_err_t init_i2c_master(void);

esp_err_t mpu9250_init(void);

esp_err_t mpu9250_read(void);

void mpu9250_task(void *pvParameters);

mpu9250_data_t *get_mpu9250_data(void);

#endif //MPU9250_H
