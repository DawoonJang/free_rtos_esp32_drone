# FreeRTOS EPS32 Drone Project

## Sensor

# MPU9250 Sensor Calibration

To minimize sensor noise and correct for inherent bias, we perform an initial calibration routine at startup. During
this process, we collect 1,000 samples of raw accelerometer and gyroscope data. The mean value of each axis is computed
and treated as the sensor offset.

These offset values are then subtracted from all subsequent sensor readings to ensure that the measured data reflects
true motion, rather than static bias or noise.

Purpose of Calibration
• Reduces sensor noise by averaging out short-term fluctuations
• Compensates for bias in both accelerometer and gyroscope outputs
• Improves accuracy of orientation estimation in filtering algorithms (e.g., Complementary Filter)

Implementation Highlights
• Samples are collected only once at startup.
• Calibration assumes the sensor is stationary and level during this period.
• The result is stored and used to correct all future data in real time.

# Complementary Filter

To estimate stable orientation angles (pitch and roll) from the MPU9250 sensor, we use a **Complementary Filter**,
which combines gyroscope and accelerometer data to balance out their individual weaknesses.  
The filtered angles are calculated using the following equations:

```
θ_comp_x = α × (θ_comp_x_prev + gyro_x × Δt) + (1 - α) × θ_acc_x  
θ_comp_y = α × (θ_comp_y_prev + gyro_y × Δt) + (1 - α) × θ_acc_y
```

### Variables

- `θ_comp_x`, `θ_comp_y`: Final filtered roll and pitch angles (in degrees)
- `θ_comp_x_prev`, `θ_comp_y_prev`: Previous filtered angles
- `gyro_x`, `gyro_y`: Angular velocity from the gyroscope (in degrees per second)
- `Δt`: Time elapsed since the last update (in seconds)
- `θ_acc_x`, `θ_acc_y`: Angle estimated from the accelerometer (in degrees)
- `α`: Filter coefficient (between 0 and 1, typically around 0.95–0.98)

### How it works

- The **gyroscope** provides fast and responsive angular rate data, but it **drifts** over time.
- The **accelerometer** is noisy but gives an **absolute reference** to gravity.
- The **complementary filter** blends both:
    - It integrates gyro data for responsiveness
    - It corrects drift using accelerometer data

### Example α values

| α Value | Description                                                     |
|--------:|-----------------------------------------------------------------|
|    0.98 | Heavier reliance on gyroscope (smoother, but more drift)        |
|    0.95 | Common setting, balances both sources                           |
|    0.90 | Faster correction using accelerometer (more responsive to tilt) |