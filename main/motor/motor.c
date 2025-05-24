#include "motor.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu.h"

static const char *TAG = "MOTOR";

const double Kp = 2.0;
const double Kd = 0.5;
const double Ki = 0.01;

double	target_angle_x, target_angle_y, target_angle_z;
uint32_t throttle;

esp_err_t motor_init()
{
	 const int				 motor_pins[]		= {MOTOR_PIN_LU, MOTOR_PIN_RU, MOTOR_PIN_RL, MOTOR_PIN_LL};
	 const ledc_channel_t motor_channels[] = {MOTOR_CHANNEL_LU, MOTOR_CHANNEL_RU, MOTOR_CHANNEL_RL, MOTOR_CHANNEL_LL};

	 // set timer
	 const ledc_timer_config_t ledc_timer = {.speed_mode		 = MOTOR_MODE,
														  .duty_resolution = MOTOR_DUTY_RES,
														  .timer_num		 = MOTOR_TIMER,
														  .freq_hz			 = MOTOR_FREQ,
														  .clk_cfg			 = LEDC_AUTO_CLK};

	 esp_err_t err = ledc_timer_config(&ledc_timer);

	 if (err != ESP_OK)
		  return err;

	 // set channel
	 for (int i = 0; i < 4; i++)
	 {
		  ledc_channel_config_t ledc_channel = {.speed_mode = MOTOR_MODE,
															 .channel	 = motor_channels[i],
															 .timer_sel	 = MOTOR_TIMER,
															 .intr_type	 = LEDC_INTR_DISABLE,
															 .gpio_num	 = motor_pins[i],
															 .duty		 = LEDC_DUTY_INIT,
															 .hpoint		 = 0};

		  err = ledc_channel_config(&ledc_channel);
		  if (err != ESP_OK)
				return err;
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
	 const ledc_channel_t motor_channels[] = {MOTOR_CHANNEL_LU, MOTOR_CHANNEL_RU, MOTOR_CHANNEL_RL, MOTOR_CHANNEL_LL};

	 for (int i = 0; i < 4; i++)
	 {
		  accel_motor(motor_channels[i], (uint32_t) 0); // 듀티 0으로 설정
	 }
}

void auto_aviation_task(void *pvParameters)
{
	 vTaskDelay(pdMS_TO_TICKS(1000));

	 for (int thr = 0; thr < 475; thr++)
	 { // 475*4 = 1900밀리 초간 상승
		  throttle = thr;
		  vTaskDelay(pdMS_TO_TICKS(4));
	 }

	 for (int thr = 475; thr > 300; thr--)
	 { // (475-300)*12 = 2100밀리 초간 하강
		  throttle = thr;
		  vTaskDelay(pdMS_TO_TICKS(12));
	 }

	 for (int thr = 300; thr > 0; thr--)
	 { // 300*5 = 1500밀리 초간 하강
		  throttle = thr;
		  vTaskDelay(pdMS_TO_TICKS(5));
	 }

	 vTaskDelete(NULL);
}

void motor_task(void *pvParameters)
{
	 static double	  integral_term_angle_x, integral_term_angle_y, integral_term_angle_z;
	 mpu9250_data_t *p_mpu9250_data = get_mpu9250_data();

	 while (1)
	 {
		  if (throttle == 0)
		  {
				p_mpu9250_data->complemented_angle_x = 0;
				p_mpu9250_data->complemented_angle_y = 0;
				p_mpu9250_data->complemented_angle_z = 0;
		  }

		  // PID Control
		  const double error_angle_x = target_angle_x - p_mpu9250_data->complemented_angle_x;
		  const double error_angle_y = target_angle_y - p_mpu9250_data->complemented_angle_y;
		  const double error_angle_z = target_angle_z - p_mpu9250_data->complemented_angle_z;

		  double balance_correction_x = Kp * error_angle_x + Kd * -p_mpu9250_data->calibrated_angular_velocity_x;
		  double balance_correction_y = Kp * error_angle_y + Kd * -p_mpu9250_data->calibrated_angular_velocity_y;
		  double balance_correction_z = Kp * error_angle_z + Kd * -p_mpu9250_data->calibrated_angular_velocity_z;

		  if (throttle == 0)
		  {
				balance_correction_x = balance_correction_y = balance_correction_z = 0.0;
		  }

		  integral_term_angle_x += Ki * error_angle_x * p_mpu9250_data->delta_t;
		  integral_term_angle_y += Ki * error_angle_y * p_mpu9250_data->delta_t;
		  integral_term_angle_z += Ki * error_angle_z * p_mpu9250_data->delta_t;

		  integral_term_angle_x = CLAMP(integral_term_angle_x, -INTEGRAL_TERM_LIMIT, INTEGRAL_TERM_LIMIT);
		  integral_term_angle_y = CLAMP(integral_term_angle_y, -INTEGRAL_TERM_LIMIT, INTEGRAL_TERM_LIMIT);
		  integral_term_angle_z = CLAMP(integral_term_angle_z, -INTEGRAL_TERM_LIMIT, INTEGRAL_TERM_LIMIT);

		  if (throttle == 0)
		  {
				integral_term_angle_x = integral_term_angle_y = integral_term_angle_z = 0.0;
		  }

		  balance_correction_x += integral_term_angle_x;
		  balance_correction_y += integral_term_angle_y;
		  balance_correction_z += integral_term_angle_z;

		  int32_t speed_lu_motor = throttle + balance_correction_x - balance_correction_y + balance_correction_z;
		  int32_t speed_ru_motor = throttle - balance_correction_x - balance_correction_y - balance_correction_z;
		  int32_t speed_rl_motor = throttle - balance_correction_x + balance_correction_y + balance_correction_z;
		  int32_t speed_ll_motor = throttle + balance_correction_x + balance_correction_y - balance_correction_z;


		  speed_lu_motor = CLAMP(speed_lu_motor, 0, 1000);
		  speed_ru_motor = CLAMP(speed_ru_motor, 0, 1000);
		  speed_ll_motor = CLAMP(speed_ll_motor, 0, 1000);
		  speed_rl_motor = CLAMP(speed_rl_motor, 0, 1000);

		  accel_motor(MOTOR_CHANNEL_LU, speed_lu_motor);
		  accel_motor(MOTOR_CHANNEL_RU, speed_ru_motor);
		  accel_motor(MOTOR_CHANNEL_RL, speed_rl_motor);
		  accel_motor(MOTOR_CHANNEL_LL, speed_ll_motor);

		  vTaskDelay(pdMS_TO_TICKS(10));
	 }
}
