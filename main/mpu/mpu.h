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

typedef struct
{
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temp;
} mpu9250_data_t;


esp_err_t init_i2c_master(void);

esp_err_t mpu9250_init(void);

esp_err_t mpu9250_read(mpu9250_data_t *data);

#endif //MPU9250_H
