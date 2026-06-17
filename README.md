# Two-Wheeled Self-Balancing Robot

> **HTX Physics A — Independent Project**  
> Hamza Alomari · 3.B · Frederikshavn Teknisk Gymnasium  
> Supervisor: Kasper Weirum Risgaard · March–April 2026

A two-wheeled self-balancing robot modelled as an **inverted pendulum**, controlled by a **PID regulator** and driven by sensor fusion from a **MPU-6050/9250 IMU** using a complementary filter.

---

## Physics Overview

The robot is an inverted pendulum. Using **Lagrangian mechanics**, the equation of motion for the system is derived as:

```
θ̈ = (mg·lc / I) · sin(θ)
```

For small angles (`|θ| ≲ 15°`), this linearises to:

```
θ̈ = (mg·lc / I) · θ
```

The general solution shows exponential growth with growth rate `λ = √(mg·lc / I)`, confirming the system is **inherently unstable** and requires active control.

For a robot with `l = 0.20 m`, the tilt angle doubles approximately every **0.14 s** — demanding a fast-reacting controller.

---

## Sensor Fusion — Complementary Filter

The IMU provides two independent but flawed angle estimates:

| Source | Strength | Weakness |
|--------|----------|----------|
| **Accelerometer** | Accurate long-term | Noisy during motion |
| **Gyroscope** | Precise short-term | Drifts over time |

The complementary filter combines both:

```
θ(t) = α · [θ(t-1) + ω·Δt] + (1 - α) · θ_acc
```

With `α = 0.98`, the gyroscope dominates on fast timescales while the accelerometer corrects for drift.

---

## PID Controller

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)
```

### Tuned parameters (Experiment 2)

| Parameter | Value | Role |
|-----------|-------|------|
| `Kp` | 15.0 | Proportional — main driving force |
| `Ki` | 0.2  | Integral — eliminates steady-state error |
| `Kd` | 10.0 | Derivative — damps oscillations |

---

## Hardware

| Component | Specification |
|-----------|--------------|
| Microcontroller | Arduino Nano |
| IMU | MPU-9250 / MPU-6050 |
| Motor driver | L298N Dual H-Bridge |
| Motors | 2× TT DC motor, 6–12 V |
| Power | 9 V battery |
| Capacitors | 1000 µF / 100 µF / 0.1 µF (noise filtering) |

---


## Experiments & Results

### Experiment 1 — IMU Characterisation

Three angle estimation methods were compared at a known angle of `0.06°`:

- **Gyroscope only** → linear drift accumulates over time (integration error)
- **Accelerometer only** → noisy but stable around the true angle
- **Complementary filter (α = 0.98)** → closely tracks the real angle ✅

### Experiment 2 — PID Tuning

With the tuned PID parameters, the robot maintained stable upright balance with oscillations of **±2°** around the setpoint (0°).

### Experiment 3 — Maximum Recovery Angle

The robot was manually tilted to various angles and released:

| Disturbance | Outcome |
|-------------|---------|
| 2.5° | ✅ Recovered |
| 3.0° | ✅ Recovered |
| 15.5° | ✅ Recovered |
| 20.0° | ❌ Fell over |

**Maximum recovery angle: `15° < θ_max < 20°`**

At 20°+, the non-linear term `sin(θ)` dominates and the linearised PID model breaks down, causing exponential divergence.

---

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software)
- Arduino Nano (or compatible board)
- MPU-6050 or MPU-9250 IMU
- L298N motor driver + 2× DC motors

### Upload

1. Clone this repository:
   ```bash
   git clone https://github.com/YOUR_USERNAME/self-balancing-robot.git
   ```

2. Open `src/imu_characterization/imu_characterization.ino` in Arduino IDE.

3. Select **Arduino Nano** and the correct COM port.

4. Upload and open the Serial Monitor at **115200 baud**.

5. Once the IMU is verified, upload `src/balance_controller/balance_controller.ino`.

6. Adjust `PITCH_OFFSET` if the robot leans slightly when stationary.

### Wiring

| IMU Pin | Arduino Nano Pin |
|---------|-----------------|
| VCC | 3.3 V or 5 V* |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

 **Important:** Some MPU modules have **no onboard voltage regulator** and accept **max 3.3 V**. Check your module's datasheet before connecting to 5 V.

---

## Theory References

- Morin, D. (2007). *The Lagrangian Method*. Harvard Physics Department.
- InvenSense (2012). *MPU-6050 Register Map and Descriptions, Rev. 4.0*.
- InvenSense (2014). *MPU-9250 Product Specification*.
- HiBit (2023). [Complementary Filter and Relative Orientation with MPU-6050](https://www.hibit.dev/posts/92/complementary-filter-and-relative-orientation-with-mpu6050)
- microcontrollerslab (2019). [PID Controller Implementation using Arduino](https://microcontrollerslab.com/pid-controller-implementation-using-arduino/)

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

*Project completed April 2026 — Frederikshavn Teknisk Gymnasium, HTX*
