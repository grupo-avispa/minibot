/*
 * Minibot — autonomous differential-drive robot for ROS 2 teaching
 *
 * Copyright (c) 2026 Óscar Pons Fernández
 * Copyright (c) 2026 José Galeas
 * Copyright (c) 2026 Alberto J. Tudela Roldán
 * Copyright (c) 2026 Guillermo Martos
 * Grupo Avispa, DTE, Universidad de Málaga
 *
 * This file is part of the Minibot project.
 *
 * Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
 * 4.0 International License (CC BY-NC-SA 4.0). You may use, share, and adapt
 * this work for non-commercial purposes, with attribution, distributing any
 * derivative works under the same license. Commercial use requires prior
 * written permission from the authors.
 *
 * You should have received a copy of the license with this project (see the
 * LICENSE file). If not, see: https://creativecommons.org/licenses/by-nc-sa/4.0/
 */
#ifndef BMI160_SENSOR_HPP
#define BMI160_SENSOR_HPP

#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>

// micro-ROS includes
#include <rcl/rcl.h>
#include <sensor_msgs/msg/imu.h>
#include <tf2_msgs/msg/tf_message.h>

class Bmi160Sensor {
public:
    Bmi160Sensor();
    void init();
    void timer_imu_callback(rcl_timer_t * timer, int64_t last_call_time);

    int64_t ros_synced_time_ms = 0;
    long ms_before_sync = 0;

    rcl_publisher_t imu_pub;
    rcl_timer_t timer_imu;
    sensor_msgs__msg__Imu imu_msg;

    rcl_publisher_t tf_pub;
    tf2_msgs__msg__TFMessage tf_msg;

private:
    BMI160 IMU;
    calData calib = { 0 };
};

#endif // BMI160_SENSOR_HPP