# Minibot

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](LICENSE)
[![PlatformIO](https://img.shields.io/badge/built%20with-PlatformIO-orange.svg?logo=platformio&logoColor=white)](https://platformio.org/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-WiFi-blue.svg)](https://micro.ros.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Arduino-E7352C.svg?logo=espressif&logoColor=white)](datasheets/)

An open, low-cost **differential-drive autonomous robot** built around the **ESP32** and **micro-ROS**, designed as a teaching platform for **ROS 2**. The firmware streams LiDAR scans, IMU data, and wheel odometry to a ROS 2 host over WiFi and drives the robot from `cmd_vel` commands, so students can experiment with the full SLAM → localization → navigation stack (SLAM Toolbox + Nav2) on real hardware.

Developed by the **AVISPA** research group, Department of Electronic Technology, University of Málaga.

![Minibot](docs/robot.png)

## Features

- **micro-ROS over WiFi** — the ESP32 registers as ROS 2 nodes through a micro-ROS Agent, no USB tether required.
- **Differential drive** with two JGA25-371 DC gearmotors, Hall encoders, and a **dual PID velocity loop** for accurate odometry.
- **Sensor fusion** — wheel odometry plus a BMI160 IMU (`/imu/data_raw`) for robust pose estimation.
- **2D LiDAR** — a 3irobotix Delta-2A parsed on-device and published as `sensor_msgs/LaserScan`, with the mirror-inversion and zero-angle defects fixed in the driver.
- **Human-robot interaction** — animated OLED eyes, an RGB status LED, and DFPlayer voice feedback, driven from a `/robot_emotion` topic.
- **FreeRTOS multitasking** — separate tasks for LiDAR reading, scan publishing, and the micro-ROS executor, pinned across both ESP32 cores.
- **3D-printable chassis** — Fusion 360 source, STEP, and ready-to-print STL files, plus a custom KiCad PCB.

## Repository layout

| Path                      | Description                                                                                  |
| ------------------------- | -------------------------------------------------------------------------------------------- |
| [source/](source)         | micro-ROS firmware (single PlatformIO project): motors, encoders, IMU, LiDAR, and HRI module. |
| [designs/](designs)       | Mechanical design: Fusion 360 (`.f3z`), STEP, and printable STLs.                            |
| [schematics/](schematics) | KiCad project for the control PCB (schematic, layout, gerbers, 3D STEP).                     |
| [datasheets/](datasheets) | Component datasheets (ESP32, JGA25-371, DRV8871, BMI160, LM2596S).                           |
| [docs/](docs)             | Images and documentation assets.                                                            |

### Firmware modules — [source/](source)

| Module                                                                                | Responsibility                                                              |
| ------------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| [jga_motors](source/src/jga_motors.cpp)         | DC motor PID control, encoder reading, odometry and `/tf`.                  |
| [bmi160_sensor](source/src/bmi160_sensor.cpp)   | BMI160 IMU acquisition and `/imu/data_raw` publishing.                      |
| [xiaomi_lidar](source/src/xiaomi_lidar.cpp)     | Delta-2A frame parsing and `/scan` publishing.                             |
| [emotions](source/src/emotions.cpp)             | OLED eyes, RGB LED, and voice feedback driven from `/robot_emotion`.        |
| [main.cpp](source/src/main.cpp)                 | micro-ROS entities, time sync, FreeRTOS tasks, and reconnection handling.  |

## Hardware

- **MCU board:** ESP32-WROOM-32 — ESP32 DevKit C V4 (USB-C), PlatformIO board `az-delivery-devkit-v4`
- **Motors:** 2× JGA25-371 DC gearmotors with Hall quadrature encoders
- **Motor driver:** DRV8871 H-bridge (per motor), 2-pin PWM control
- **LiDAR:** 3irobotix Delta-2A 2D scanner (≈0.13–8 m), UART TTL 3.3 V
- **IMU:** Bosch BMI160 6-axis (I²C, address `0x69`)
- **Power:** 3S Li-Ion 18650 pack with BMS (≈11.1 V) → 12 V to motors; LM2596S buck converter → 5 V rail for ESP32 and LiDAR
- **HRI:** 2× SSD1306 OLED displays, RGB LED, DFPlayer Mini audio
- **Chassis & PCB:** 3D-printed body (PLA + TPU, see [designs/](designs)) on a custom KiCad PCB (see [schematics/](schematics))

Datasheets for the main components are in [datasheets/](datasheets).

### Pinout (ESP32 GPIO)

| Subsystem        | Signal                        | GPIO                    |
| ---------------- | ----------------------------- | ----------------------- |
| Left motor       | IN1 / IN2 (PWM)               | 21 / 19                 |
| Left encoder     | A / B                         | 32 / 33                 |
| Right motor      | IN1 / IN2 (PWM)               | 4 / 27                  |
| Right encoder    | A / B                         | 34 / 35                 |
| Encoder LEDs     | Red / Blue                    | 16 / 17                 |
| LiDAR            | UART RX (Serial2) / motor enable | 26 / 25              |
| IMU + OLED (I²C) | SDA / SCL                     | 13 / 14                 |
| RGB LED          | R / G / B                     | 15 / 12 / 22            |
| DFPlayer (audio) | RX / TX                       | 18 / 5                  |

Pin assignments are defined at the top of each module ([jga_motors.hpp](source/include/jga_motors.hpp), [emotions.hpp](source/include/emotions.hpp)) and in the corresponding `.cpp` files.

### Robot constants

- Wheel radius: `0.0427 m`
- Wheel base (track width): `0.2195 m`
- Encoder ticks per wheel revolution: `1980`
- Speed range: `0.08 – 0.3 m/s`
- PID gains (per wheel): `kp = 0.8`, `ki = 0.4`, `kd = 0.1`

## Requirements

- [PlatformIO](https://platformio.org/) (CLI or the VS Code extension)
- A ROS 2 installation on the host (firmware targets **Jazzy**)
- A running **micro-ROS Agent** on the host

## Configuration

Edit the WiFi credentials and the micro-ROS Agent endpoint in [source/config/uros_wifi_config.h](source/config/uros_wifi_config.h):

```cpp
char ssid[] = "your-wifi-ssid";
char psk[]  = "your-wifi-password";

IPAddress agent_ip(192, 168, 0, 100);   // host running the micro-ROS Agent
size_t    agent_port = 8888;
```

Review the hardware pins (see [pinout](#pinout-esp32-gpio) above) and, if needed, the robot dimensions and PID gains in [jga_motors.hpp](source/include/jga_motors.hpp).

The firmware ships a [custom.meta](source/custom.meta) that raises the micro-XRCE-DDS transport MTU and entity limits — keep it in sync if you add publishers/subscriptions.

## Build, flash and monitor

[source/](source) is a PlatformIO project. From that directory:

```bash
cd source

pio run                        # compile
pio run -t upload              # flash over USB
pio device monitor -b 115200   # open the serial monitor
```

## Running with ROS 2

1. Start the micro-ROS Agent on your ROS 2 host (matching the IP/port configured in firmware):

   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

2. Power on the ESP32. It connects to WiFi, registers its nodes with the Agent, and synchronizes its clock with the host. If the Agent becomes unreachable it stops the motors and recreates its ROS entities automatically.
3. Inspect the graph:

   ```bash
   ros2 topic list
   ros2 topic echo /scan
   ```

### Topics and frames

Nodes: `minibot_lidar` and `minibot_jga`.

| Direction | Topic            | Type                    |
| --------- | ---------------- | ----------------------- |
| Publish   | `/scan`          | `sensor_msgs/LaserScan` |
| Publish   | `/odom`          | `nav_msgs/Odometry`     |
| Publish   | `/imu/data_raw`  | `sensor_msgs/Imu`       |
| Publish   | `/tf`            | `tf2_msgs/TFMessage`    |
| Subscribe | `/cmd_vel`       | `geometry_msgs/Twist`   |
| Subscribe | `/robot_emotion` | `std_msgs/Int32`        |

Frames: `odom` → `base_link`, plus `base_laser_link` for the scan.

Drive the robot with, for example:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Mechanical design

3D models live in [designs/](designs):

- `Fusion/Robotito v3.f3z` — editable Fusion 360 source
- `STEP/Robotito v3.step` — neutral CAD exchange format
- `STLs/` — printable parts (main body, wheels and tires, caster wheel + support, motor clamps, LiDAR support, PCB support and spacers, front face)

The control PCB (KiCad project, gerbers, and a 3D STEP export) is in [schematics/](schematics).

## Implementation notes

- The firmware runs dedicated FreeRTOS tasks for LiDAR serial parsing, `LaserScan` assembly/publishing, and the micro-ROS executor, pinned across both cores.
- Motor speed is closed-loop: encoder ticks feed a per-wheel PID with a state machine for smooth start/stop and braking.
- Odometry is integrated from encoder counts and published with the `odom` → `base_link` TF; the BMI160 adds orientation data on `/imu/data_raw`.
- The Delta-2A driver corrects the original mirror inversion and uses a start-angle zero-crossing detector for consistent scan framing.
- A periodic sync timer aligns the ESP32 clock with the micro-ROS Agent so message timestamps are valid on the host.

## License

This project — firmware, mechanical designs, schematics, and documentation — is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0)**.

You are free to **share** and **adapt** the material for **non-commercial** purposes, provided you give appropriate **credit** and license your derivatives under the **same** terms. Commercial use requires prior written permission from the author.

See [LICENSE](LICENSE) for the full text · [License summary](https://creativecommons.org/licenses/by-nc-sa/4.0/)
