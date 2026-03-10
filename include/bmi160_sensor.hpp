#ifndef BMI160_SENSOR_HPP
#define BMI160_SENSOR_HPP

#include <Arduino.h>
#include <Wire.h>
#include <FastIMU.h>

// micro-ROS includes
#include <rcl/rcl.h>
#include <sensor_msgs/msg/imu.h>

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

private:
    BMI160 IMU;
    calData calib = { 0 };
};

#endif // BMI160_SENSOR_HPP