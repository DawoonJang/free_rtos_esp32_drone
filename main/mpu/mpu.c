#include "mpu.h"
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU";

mpu9250_data_t sensor_mpu9250_data;

esp_err_t init_i2c_master(void)
{
	 i2c_config_t conf = {
			 .mode				 = I2C_MODE_MASTER,
			 .sda_io_num		 = I2C_MASTER_SDA_IO,
			 .scl_io_num		 = I2C_MASTER_SCL_IO,
			 .sda_pullup_en	 = GPIO_PULLUP_ENABLE,
			 .scl_pullup_en	 = GPIO_PULLUP_ENABLE,
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

	 esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_ADDR, data, sizeof(data),
															  I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

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

static inline int16_t combine_two_bytes(uint8_t msb_data, uint8_t lsb_data)
{
	 return (int16_t) ((msb_data << 8) | lsb_data);
}

static esp_err_t s_mpu9250_read_raw(void)
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

	 sensor_mpu9250_data.acceleration_x_raw	  = combine_two_bytes(raw_data[0], raw_data[1]);
	 sensor_mpu9250_data.acceleration_y_raw	  = combine_two_bytes(raw_data[2], raw_data[3]);
	 sensor_mpu9250_data.acceleration_z_raw	  = combine_two_bytes(raw_data[4], raw_data[5]);
	 sensor_mpu9250_data.temp						  = combine_two_bytes(raw_data[6], raw_data[7]);
	 sensor_mpu9250_data.angular_velocity_x_raw = combine_two_bytes(raw_data[8], raw_data[9]);
	 sensor_mpu9250_data.angular_velocity_y_raw = combine_two_bytes(raw_data[10], raw_data[11]);
	 sensor_mpu9250_data.angular_velocity_z_raw = combine_two_bytes(raw_data[12], raw_data[13]);
	 sensor_mpu9250_data.temp						  = (sensor_mpu9250_data.temp / 340.0f) + 36.53f;

	 return ESP_OK;
}

static esp_err_t _mpu9250_calibrate(void)
{
	 static uint16_t caliration_cnt;

	 if (caliration_cnt > CALIBRATE_CNT)
	 {
		  return ESP_OK;
	 }

	 if (s_mpu9250_read_raw() != ESP_OK)
	 {
		  return ESP_FAIL;
	 }

	 sensor_mpu9250_data.acc_x_offset =
			 (sensor_mpu9250_data.acc_x_offset * caliration_cnt + (double) sensor_mpu9250_data.acceleration_x_raw) /
			 (caliration_cnt + 1);

	 sensor_mpu9250_data.acc_y_offset =
			 (sensor_mpu9250_data.acc_y_offset * caliration_cnt + (double) sensor_mpu9250_data.acceleration_y_raw) /
			 (caliration_cnt + 1);

	 sensor_mpu9250_data.acc_z_offset =
			 (sensor_mpu9250_data.acc_z_offset * caliration_cnt + (double) sensor_mpu9250_data.acceleration_z_raw) /
			 (caliration_cnt + 1);

	 sensor_mpu9250_data.angular_velocity_x_offset = (sensor_mpu9250_data.angular_velocity_x_offset * caliration_cnt +
																	  (double) sensor_mpu9250_data.angular_velocity_x_raw) /
																	 (caliration_cnt + 1);

	 sensor_mpu9250_data.angular_velocity_y_offset = (sensor_mpu9250_data.angular_velocity_y_offset * caliration_cnt +
																	  (double) sensor_mpu9250_data.angular_velocity_y_raw) /
																	 (caliration_cnt + 1);

	 sensor_mpu9250_data.angular_velocity_z_offset = (sensor_mpu9250_data.angular_velocity_z_offset * caliration_cnt +
																	  (double) sensor_mpu9250_data.angular_velocity_z_raw) /
																	 (caliration_cnt + 1);
	 caliration_cnt++;

	 return ESP_FAIL;
}

mpu9250_data_t *get_mpu9250_data(void) { return &sensor_mpu9250_data; }

esp_err_t mpu9250_read(void)
{
	 if (_mpu9250_calibrate() == ESP_FAIL)
	 {
		  return ESP_ERR_INVALID_STATE;
	 }

	 const esp_err_t ret = s_mpu9250_read_raw();

	 if (ret != ESP_OK)
		  return ret;

	 sensor_mpu9250_data.acceleration_x_raw -= (int16_t) sensor_mpu9250_data.acc_x_offset;
	 sensor_mpu9250_data.acceleration_y_raw -= (int16_t) sensor_mpu9250_data.acc_y_offset;
	 sensor_mpu9250_data.acceleration_z_raw -= (int16_t) sensor_mpu9250_data.acc_z_offset;

	 sensor_mpu9250_data.angular_velocity_x_raw -= (int16_t) sensor_mpu9250_data.angular_velocity_x_offset;
	 sensor_mpu9250_data.angular_velocity_y_raw -= (int16_t) sensor_mpu9250_data.angular_velocity_y_offset;
	 sensor_mpu9250_data.angular_velocity_z_raw -= (int16_t) sensor_mpu9250_data.angular_velocity_z_offset;

	 return ESP_OK;
}

void mpu9250_task(void *pvParameters)
{
	 static int64_t t_prev;
	 static double	 angle_x, angle_y, angle_z;

	 while (_mpu9250_calibrate() == ESP_FAIL)
	 {
		  vTaskDelay(pdMS_TO_TICKS(2));
	 }

	 t_prev = esp_timer_get_time();

	 while (1)
	 {
		  const esp_err_t ret = mpu9250_read();

		  if (ret == ESP_OK)
		  {
				const int64_t t_now			 = esp_timer_get_time();
				sensor_mpu9250_data.delta_t = (double) (t_now - t_prev) / 1000000.0; // 초 단위
				t_prev							 = t_now;

				sensor_mpu9250_data.calibrated_angular_velocity_x =
						(float) sensor_mpu9250_data.angular_velocity_x_raw / GYROXYZ_TO_DEGREES_PER_SEC;
				sensor_mpu9250_data.calibrated_angular_velocity_y =
						(float) sensor_mpu9250_data.angular_velocity_y_raw / GYROXYZ_TO_DEGREES_PER_SEC;
				sensor_mpu9250_data.calibrated_angular_velocity_z =
						(float) sensor_mpu9250_data.angular_velocity_z_raw / GYROXYZ_TO_DEGREES_PER_SEC;

				angle_x += sensor_mpu9250_data.calibrated_angular_velocity_x * sensor_mpu9250_data.delta_t;
				angle_y += sensor_mpu9250_data.calibrated_angular_velocity_y * sensor_mpu9250_data.delta_t;
				angle_z += sensor_mpu9250_data.calibrated_angular_velocity_z * sensor_mpu9250_data.delta_t;

				const double acceleration_yz_magnitude =
						sqrt(pow(sensor_mpu9250_data.acceleration_y_raw, 2) + pow(sensor_mpu9250_data.acceleration_z_raw, 2));
				const double acceleration_xz_magnitude =
						sqrt(pow(sensor_mpu9250_data.acceleration_x_raw, 2) + pow(sensor_mpu9250_data.acceleration_z_raw, 2));

				const double acceleration_angle_y =
						atan(-sensor_mpu9250_data.acceleration_x_raw / acceleration_yz_magnitude) * RADIANS_TO_DEGREES;
				const double acceleration_angle_x =
						atan(sensor_mpu9250_data.acceleration_y_raw / acceleration_xz_magnitude) * RADIANS_TO_DEGREES;
				// const double acAngleZ = 0.0; // 가속도만으로는 Z는 알 수 없음

				// Complementary Filter
				sensor_mpu9250_data.complemented_angle_x =
						ALPHA * (sensor_mpu9250_data.complemented_angle_x +
									sensor_mpu9250_data.calibrated_angular_velocity_x * sensor_mpu9250_data.delta_t) +
						(1.0 - ALPHA) * acceleration_angle_x;
				sensor_mpu9250_data.complemented_angle_y =
						ALPHA * (sensor_mpu9250_data.complemented_angle_y +
									sensor_mpu9250_data.calibrated_angular_velocity_y * sensor_mpu9250_data.delta_t) +
						(1.0 - ALPHA) * acceleration_angle_y;
				sensor_mpu9250_data.complemented_angle_z = angle_z;

				// SET LOG
				ESP_LOGI(TAG, "angular_velocity X:%.2f Y:%.2f Z:%.2f | angke X:%.2f Y:%.2f Z:%.2f",
							sensor_mpu9250_data.calibrated_angular_velocity_x,
							sensor_mpu9250_data.calibrated_angular_velocity_y,
							sensor_mpu9250_data.calibrated_angular_velocity_z, sensor_mpu9250_data.complemented_angle_x,
							sensor_mpu9250_data.complemented_angle_y, sensor_mpu9250_data.complemented_angle_z);
		  }
		  else
		  {
				ESP_LOGE(TAG, "MPU9250 read failed: %s", esp_err_to_name(ret));
		  }

		  vTaskDelay(pdMS_TO_TICKS(500));
	 }
}
