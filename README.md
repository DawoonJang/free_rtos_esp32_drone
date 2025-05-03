# FreeRTOS EPS32 Drone Project

# FreeRTOS ESP32 Drone Project

## Sensor

### MPU9250 Sensor Calibration

To minimize sensor noise and correct inherent bias, an initial calibration routine is executed at startup. During this
process, the system collects **1,000 samples** of raw accelerometer and gyroscope data. The mean of each axis is
computed and treated as the sensor's offset:

```
acc_offset  = (∑ₜ₌₀⁹⁹⁹ acc_t)  / N
gyro_offset = (∑ₜ₌₀⁹⁹⁹ gyro_t) / N

Once `t ≥ 1000`, raw sensor data is calibrated as follows:
calibrated_acc_t  = acc_t  - acc_offset
calibrated_gyro_t = gyro_t - gyro_offset
```

These offsets are subtracted from all subsequent readings to ensure the data reflects true motion rather than static
bias or noise.

#### Purpose of Calibration

- Averages out short-term fluctuations to **reduce sensor noise**
- Compensates for **bias in both accelerometer and gyroscope**
- Improves the **accuracy of orientation estimation** in filtering algorithms such as the Complementary Filter

#### Implementation Highlights

- Samples are collected **once at startup**
- Assumes the sensor is **stationary and level** during this period
- Calculated offsets are stored and used to **correct all future sensor data in real time**

---

### Complementary Filter

To estimate stable orientation angles (pitch and roll) using the MPU9250, a **Complementary Filter** is used. This
filter fuses gyroscope and accelerometer data, leveraging the strengths of each.

Filtered angles are calculated as:

```
θₓ(t) = α × [θₓ(t - 1) + gyroₓ × Δt] + (1 - α) × θ_accₓ
θᵧ(t) = α × [θᵧ(t - 1) + gyroᵧ × Δt] + (1 - α) × θ_accᵧ
```

#### Variables

- `θₓ(t), θᵧ(t)`: Filtered roll and pitch angles (degrees)
- `θₓ(t - 1), θᵧ(t - 1)`: Previous filtered angles
- `gyroₓ, gyroᵧ`: Angular velocity from the gyroscope (°/s)
- `Δt`: Time elapsed since the last update (seconds)
- `θ_accₓ, θ_accᵧ`: Angle estimated from the accelerometer (degrees)
- `α`: Filter coefficient (range: 0 < α < 1; typically 0.95–0.98)

#### How It Works

- The **gyroscope** provides smooth and responsive angular rate data, but accumulates **drift over time**
- The **accelerometer** is noisy but provides an **absolute reference to gravity**
- The **Complementary Filter** blends both:
    - **Gyro data** is integrated for responsiveness
    - **Accel data** corrects for long-term drift

#### Common α Values

| α Value | Behavior Description                                              |
|--------:|-------------------------------------------------------------------|
|    0.98 | High reliance on gyro (smooth output, more drift)                 |
|    0.95 | Balanced fusion (commonly used)                                   |
|    0.90 | More responsive to tilt (faster correction, more accel influence) |

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