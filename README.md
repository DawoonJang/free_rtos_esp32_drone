# FreeRTOS EPS32 Drone Project

## MPU9250 Sensor

### 1. Offset Calibration

To minimize sensor noise and correct inherent bias, an initial calibration routine is executed at startup. During this
process, the system collects **1,000 samples** of raw accelerometer and gyroscope data. The mean of each axis is
computed and treated as the sensor's offset:

```
acc_offset  = (∑ₜ₌₀⁹⁹⁹ accₜ)  / N  
angular_velocity_offset = (∑ₜ₌₀⁹⁹⁹ angular_velocityₜ) / N  

Once t ≥ 1000, raw sensor data is calibrated as follows:  
calibrated_accₜ  = accₜ  - acc_offset  
calibrated_angular_velocityₜ = angular_velocityₜ - angular_velocity_offset
```

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