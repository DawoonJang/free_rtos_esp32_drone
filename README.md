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

#### 5. Control: PID-based Attitude Stabilization

To maintain stable flight, the drone uses a PID controller (Proportional–Integral–Derivative) to correct its orientation based on the estimated angles from the complementary filter.

The goal is to minimize the error between the target angles (e.g., from remote input or flight plan) and the current estimated angles.

(1) Error Calculation
For each axis (X, Y, Z), compute the difference between the target angle and the estimated angle:

```
errorₓ = target_angleₓ − estimated_angleₓ  
errorᵧ = target_angleᵧ − estimated_angleᵧ  
error𝓏 = target_angle𝓏 − estimated_angle𝓏
```

(2) PID Control Logic

The PID controller calculates a correction value using the following terms:

```
P-term: Kp × error (proportional to current error)  
I-term: Ki × ∫error dt (accumulates past error over time)  
D-term: Kd × d(error)/dt (based on angular velocity)
```

The full control signal for each axis is:

```
controlₓ = Kp × errorₓ + Ki × ∫errorₓ dt − Kd × gyroₓ  
controlᵧ = Kp × errorᵧ + Ki × ∫errorᵧ dt − Kd × gyroᵧ  
control𝓏 = Kp × error𝓏 + Ki × ∫error𝓏 dt − Kd × gyro𝓏
```

The integral term is reset when the throttle is zero to prevent integral windup during idle state.

(3) Motor Mixing

The control signals are combined with the base throttle to determine the PWM duty for each motor:

```
motorₐ = throttle + controlₓ − controlᵧ + control𝓏  
motorᵦ = throttle − controlₓ − controlᵧ − control𝓏  
motor𝒸 = throttle − controlₓ + controlᵧ + control𝓏  
motor𝒹 = throttle + controlₓ + controlᵧ − control𝓏
```

This mixing scheme distributes the correction forces across the quadcopter’s four motors to control pitch, roll, and yaw simultaneously.

(4) Timing

The PID control loop is executed every 10 milliseconds to ensure responsive but stable updates.