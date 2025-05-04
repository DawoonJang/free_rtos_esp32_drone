# FreeRTOS EPS32 Drone Project

# FreeRTOS ESP32 Drone Project

## MPU9250 Sensor

### 1. Offset Calibration

To minimize sensor noise and correct inherent bias, an initial calibration routine is executed at startup. During this
process, the system collects **1,000 samples** of raw accelerometer and gyroscope data. The mean of each axis is
computed and treated as the sensor's offset:

```
acc_offset  = (∑ₜ₌₀⁹⁹⁹ accₜ)  / N  
gyro_offset = (∑ₜ₌₀⁹⁹⁹ gyroₜ) / N  

Once t ≥ 1000, raw sensor data is calibrated as follows:  
calibrated_accₜ  = accₜ  - acc_offset  
calibrated_gyroₜ = gyroₜ - gyro_offset
```

---

#### 2. Gyroscope Data → Angle Calculation

The gyroscope provides **angular velocity (°/s)**, which can be integrated to obtain **angular displacement (°)**. The
code accumulates the angular velocity over time to estimate the angles of the device:

```
gyAngleₓ += gyroₓ * Δt;
gyAngleᵧ += gyroᵧ * Δt
gyAngle𝓏 += gyro𝓏 * Δt
```

However, a limitation of gyroscopes is that they suffer from **drift** over time, causing the calculated angles to
accumulate small errors as time progresses.

#### 3. Accelerometer Data → Angle Estimation

The accelerometer can be used to estimate **orientation angles** by referencing the **gravity vector**. The code
calculates **pitch (X-axis)** and **roll (Y-axis)** based on accelerometer data:

```
acAngleY = atan(-acc_x / sqrt(acc_y² + acc_z²)) * 180/π;
acAngleX = atan(acc_y / sqrt(acc_x² + acc_z²)) * 180/π;
```

The downside of the accelerometer is that it is **noisy**, especially during quick movements or vibrations, leading to
inaccurate angle estimations when the sensor is in motion.

---

#### 4. Complementary Filter: Combining Both Sensors’ Strengths

- The **gyroscope** offers **smooth and responsive** data but suffers from **long-term drift**.
- The **accelerometer** provides an **absolute reference** (gravity) but can be **noisy** and slow to react.

The **complementary filter** is used to combine the benefits of both sensors:

- `θₓ(t), θᵧ(t)`: Filtered roll and pitch angles (degrees)
- `θₓ(t - 1), θᵧ(t - 1)`: Previous filtered angles
- `gyroₓ, gyroᵧ`: Angular velocity from the gyroscope (°/s)
- `Δt`: Time elapsed since the last update (seconds)
- `θ_accₓ, θ_accᵧ`: Angle estimated from the accelerometer (degrees)
- `α`: Filter coefficient (range: 0 < α < 1; typically 0.95–0.98)

| α Value | Behavior Description                                              |
|--------:|-------------------------------------------------------------------|
|    0.98 | High reliance on gyro (smooth output, more drift)                 |
|    0.95 | Balanced fusion (commonly used)                                   |
|    0.90 | More responsive to tilt (faster correction, more accel influence) |

```
θₓ₍ₜ₎ = α × (θₓ₍ₜ₋₁₎ + gyroₓ × Δt) + (1 - α) × θ_accₓ
θᵧ₍ₜ₎ = α × [θᵧ₍ₜ₋₁₎ + gyroᵧ × Δt] + (1 − α) × θ_accᵧ
```

This filter blends the **gyro data** (for fast response) with the **accelerometer data** (for long-term stability),
helping to correct drift over time while preserving real-time responsiveness.

As a result, `sensor_mpu9250_data.complemented_angle_x` provides **a more accurate and stable angle** estimate.