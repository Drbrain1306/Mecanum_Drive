# ESP32 Closed-Loop Mecanum Drivetrain Controller

## Repo Description
A deterministic 100 Hz closed-loop Mecanum mobile robot controller implemented on the ESP32 (ESP-IDF) driving four high-power BTS7960 H-bridges via 20 kHz ultrasonic PWM, decoding 400 PPR optical encoders using native hardware pulse counters (pulse_cnt), and ingesting commands over low-latency ESP-NOW wireless links with fail-safe watchdog monitoring.

For a practical visualization, check my exampled prototype of the mecanum drive: https://clinton-mecanum.ai.studio

## Hardware Setup & Pinout
| Signal | ESP32 GPIO | Peripheral Function | Target Device Pin | Description |
| :--- | :--- | :--- | :--- | :--- |
| **M1_RPWM** | GPIO 32 | LEDC_CHANNEL_0 | Front Left IBT-2 RPWM | Forward drive PWM (20 kHz, 10-bit resolution) |
| **M1_LPWM** | GPIO 33 | LEDC_CHANNEL_1 | Front Left IBT-2 LPWM | Reverse drive PWM (20 kHz, 10-bit resolution) |
| **M1_ENCA** | GPIO 34 | PCNT Unit 0 (Edge) | Front Left Encoder A | Quadrature Phase A (*Requires external 4.7kΩ pull-up*) |
| **M1_ENCB** | GPIO 35 | PCNT Unit 0 (Level) | Front Left Encoder B | Quadrature Phase B (*Requires external 4.7kΩ pull-up*) |
| **M2_RPWM** | GPIO 25 | LEDC_CHANNEL_2 | Front Right IBT-2 RPWM | Forward drive PWM (Mirrored right-side axis) |
| **M2_LPWM** | GPIO 26 | LEDC_CHANNEL_3 | Front Right IBT-2 LPWM | Reverse drive PWM (Mirrored right-side axis) |
| **M2_ENCA** | GPIO 36 | PCNT Unit 1 (Edge) | Front Right Encoder A | Quadrature Phase A / SENSOR_VP (*External pull-up*) |
| **M2_ENCB** | GPIO 39 | PCNT Unit 1 (Level) | Front Right Encoder B | Quadrature Phase B / SENSOR_VN (*External pull-up*) |
| **M3_RPWM** | GPIO 18 | LEDC_CHANNEL_4 | Back Left IBT-2 RPWM | Forward drive PWM (20 kHz, 10-bit resolution) |
| **M3_LPWM** | GPIO 19 | LEDC_CHANNEL_5 | Back Left IBT-2 LPWM | Reverse drive PWM (20 kHz, 10-bit resolution) |
| **M3_ENCA** | GPIO 14 | PCNT Unit 2 (Edge) | Back Left Encoder A | Quadrature Phase A |
| **M3_ENCB** | GPIO 13 | PCNT Unit 2 (Level) | Back Left Encoder B | Quadrature Phase B |
| **M4_RPWM** | GPIO 23 | LEDC_CHANNEL_6 | Back Right IBT-2 RPWM | Forward drive PWM (Mirrored right-side axis) |
| **M4_LPWM** | GPIO 17 | LEDC_CHANNEL_7 | Back Right IBT-2 LPWM | Reverse drive PWM (TX2 pin; WROOM only) |
| **M4_ENCA** | GPIO 16 | PCNT Unit 3 (Edge) | Back Right Encoder A | Quadrature Phase A (RX2 pin; WROOM only) |
| **M4_ENCB** | GPIO 4 | PCNT Unit 3 (Level) | Back Right Encoder B | Quadrature Phase B |
| **DRV_EN** | GPIO 27 | GPIO Output | Common IBT-2 EN Bus | Shared R_EN / L_EN enable gate for all 4 drivers |
| **GND** | GND | Power / Signal | All Drivers / Encoders | Common system signal ground reference |

## Features & Implementation Details
- **Deterministic FreeRTOS Core Isolation**: Pinned the 100 Hz control loop task (mecanum_ctrl) to Core 1 using vTaskDelayUntil for drift-free 10ms execution, leaving Core 0 dedicated to Wi-Fi and ESP-NOW stack processing.
- **Hardware-Level 4x Quadrature Decoding**: Configured four discrete ESP32 PCNT units with dual-edge and dual-level cross-channel actions, yielding 1600 counts/revolution from 400 PPR optical encoders without interrupt CPU penalties.
- **Ultrasonic H-Bridge Drive**: Allocated 8 independent LEDC channels across LEDC_TIMER_0 at 20 kHz with 10-bit duty resolution (0–1023), eliminating motor acoustic whine and ensuring continuous bridge conduction.
- **Mecanum Inverse Kinematics with Vector Headroom Scaling**: Computed target wheel angular velocities for an O-shape roller layout while applying dynamic proportional downscaling to preserve translational heading whenever motor outputs saturate past 9.0 rad/s.
- **Low-Latency Wire Protocol**: Designed an 11-byte packed binary struct (robot_cmd_t) carrying normalized [-1000, 1000] 3-axis motion vectors, emergency stop state, and sequence tracking over unencrypted ESP-NOW frames.
- **Staleness Watchdog & Dynamic Braking**: Evaluated link freshness every loop cycle using sequence counters and tick timestamps; automatically triggers active low-side MOSFET short-circuit braking and integral state zeroing if communication ceases for >300 ms.

## Challenges Solved
- **Input-Only GPI Floating States**: Eliminated spurious pulse counts and EMI glitches on GPIO 34, 35, 36, and 39 by integrating external 4.7 kΩ pull-ups and configuring hardware glitch filters (max_glitch_ns = 1000) on the PCNT units to reject high-frequency switching noise from 24V motor leads.
- **Right-Side Geometric Mirroring Inversion**: Corrected chassis rotation faults by applying polarity inversions to both motor drive outputs and encoder counter accumulators on wheels M2 (FR) and M4 (BR), keeping global coordinate frames aligned with ROS REP-103 standards.
- **Derivative Chatter on Discrete Pulse Encoders**: Resolved high-frequency MOSFET gate jitter and motor overheating caused by raw backward-difference pulse differentiation (dt = 10 ms) by stripping raw derivative terms and stabilizing the velocity loop with feedforward voltage estimation (Kff) combined with PI feedback.
- **Failsafe Integral Windup & Reconnect Jerk**: Prevented sudden high-torque lunges when recovering from lost wireless frames by flushing accumulated integrator values (drive_state.pid_integral = 0) immediately upon timeout assertion.
