# FreeRTOS EPS32 Drone Project

## MPU9250 Sensor

### 1. Sensor Calibration: Offset Correction & Unit Conversion

To minimize sensor bias and noise, a calibration routine is executed at system startup.
This step is critical to ensure accurate orientation estimation, especially when integrating angular velocity.

(1) Step 1: Offset Estimation (Bias Correction)

The system collects 1,000 samples of raw accelerometer and gyroscope data while the device is stationary.
It computes the average (mean) value of each axis and treats it as the offset:

```
acc_offsetₓ  = (∑ₜ₌₀⁹⁹⁹ accₓ₍ₜ₎) / N  
gyro_offsetₓ = (∑ₜ₌₀⁹⁹⁹ angular_velocityₓ₍ₜ₎) / N  

Then, incoming raw data is offset-corrected in real time:
calibrated_accₓ  = raw_accₓ  − acc_offsetₓ  
calibrated_ωₓ    = raw_ωₓ    − gyro_offsetₓ

Repeat above process for Y, Z axes 
```

This step removes static bias, which would otherwise accumulate as angle drift when integrating gyro values.

(2)  Step 2: Unit Conversion (Raw → Degrees per Second)

```
calibrated_ωₓ = (float) calibrated_ωₓ / GYROXYZ_TO_DEGREES_PER_SEC  
calibrated_ωᵧ = (float) calibrated_ωᵧ / GYROXYZ_TO_DEGREES_PER_SEC  
calibrated_ω𝓏 = (float) calibrated_ω𝓏 / GYROXYZ_TO_DEGREES_PER_SEC
```

GYROXYZ_TO_DEGREES_PER_SEC is a scale factor defined by the gyroscope’s full-scale range
(e.g., 131.0 for ±250°/s, 65.5 for ±500°/s)
---

#### 2. Gyroscope Data → Angle Calculation

The gyroscope provides angular velocity (°/s), which represents how fast the device is rotating.
Since angular velocity is the rate of change of angle over time, integrating it yields the angular displacement (°).

```

angleₓ += angular_velocityₓ * Δt
angleᵧ += angular_velocityᵧ * Δt
angle𝓏 += angular_velocity𝓏 * Δt

```

However, a limitation of gyroscopes is that they suffer from **drift** over time, causing the calculated angles to
accumulate small errors as time progresses.

#### 3. Accelerometer Data → Angle Calculation

The accelerometer measures acceleration along each axis, including the force of gravity.
By referencing the gravity vector, it is possible to estimate the device’s orientation in space.

Specifically, the accelerometer can be used to estimate pitch (rotation around X-axis) and
roll (rotation around Y-axis) using trigonometric calculations:

```

angleₓ = atan2(accᵧ, √(accₓ² + acc𝓏²)) × 180 / π  
angleᵧ = atan2(−accₓ, √(accᵧ² + acc𝓏²)) × 180 / π

```

However, estimating yaw (rotation around Z-axis) is not possible using only the accelerometer,
since gravity acts vertically and does not change with horizontal (Z-axis) rotation.
While accelerometer-based angles provide an absolute reference, they are also susceptible to noise
during movement or vibration, which can lead to unstable readings when the device is not stationary.

---

#### 4. Complementary Filter → Angle Fusion

The gyroscope provides smooth and responsive angular velocity readings,
but suffers from long-term drift due to error accumulation over time.

The accelerometer offers an absolute angle reference based on gravity,
but is often noisy and unreliable during movement or vibration.

To combine the strengths of both sensors, a complementary filter is used to estimate stable orientation:

```

θₓ(t) = α × [θₓ(t−1) + ωₓ × Δt] + (1 − α) × θₐccₓ  
θᵧ(t) = α × [θᵧ(t−1) + ωᵧ × Δt] + (1 − α) × θₐccᵧ

```

Where:

```

• θₓ(t), θᵧ(t): Filtered roll and pitch angles (degrees)
• ωₓ, ωᵧ: Angular velocity from gyroscope (°/s)
• Δt: Time elapsed since last update (seconds)
• θₐccₓ, θₐccᵧ: Angle from accelerometer (degrees)
• α: Filter coefficient (0 < α < 1), controlling the fusion balance

```

This filter continuously blends the short-term precision of the gyroscope
with the long-term stability of the accelerometer,
resulting in accurate and drift-resistant angle estimates over time.

For example, complemented_angleₓ and complemented_angleᵧ can be used
for stable control feedback in embedded motion systems.