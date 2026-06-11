#ifndef NEMAMOTORS_HPP
#define NEMAMOTORS_HPP

// Motor includes
#include <AccelStepper.h>

// ROS includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>

class NemaMotors {
public:
    // Constructor
    NemaMotors();

    // Destructor
    ~NemaMotors();

    // Public methods
    void run_motors();
    void cmd_vel_sub_callback(const void *msgin);
    void timer_odometry_callback(rcl_timer_t * timer, int64_t last_call_time);
    void update_pose(const long *steps_left, const long *steps_right);
    void init();
    
    // Public attributes

    // ROS Publishers
    rcl_publisher_t odom_pub;
    rcl_publisher_t tf_pub;

    // ROS Subscriptor
    rcl_subscription_t cmd_vel_sub;

    // ROS Timer
    rcl_timer_t timer_odom;

    // ROS msgs
    nav_msgs__msg__Odometry odom_msg;
    tf2_msgs__msg__TFMessage tf_msg;
    geometry_msgs__msg__Twist cmd_vel_msg;

    // Syncronization variables
    int64_t ros_synced_time_ms = 0;
    int64_t ros_synced_time_ns = 0;
    long ms_before_sync = 0;

    AccelStepper stepper_l; // Stepper motor object for left motor
    AccelStepper stepper_r; // Stepper motor object for right motor

private:
    // Privare methods

    // Private attributes

    // Pinout configuration for motor control
    int step_pin_l;
    int dir_pin_l;
    int step_pin_r;
    int dir_pin_r;
    int motor_enable_pin; // Pin to enable/disable motors

    int micro_step; // Microstepping setting (hardware)
    int max_vel; // Maximum velocity in steps/s

    // Robot constatns for odometry
    const float wheel_radius;  // wheel radius (m)
    const float wheel_base; // distance between wheels (m)


    // Robot state variables
    float x;
    float y;
    float theta;  // radians

    // Previous steps for odometry calculation
    long prev_steps_left;
    long prev_steps_right;

    // Conversion factors
    const float steps_per_rev; // Full turn in steps (200 steps for 1.8° stepper motor, multiplied by microstepping)
    const float rad_per_step;  // 1 step = 1 / (200*micro_step) of a full turn = 2pi / (200*micro_step) rad
    const float meters_to_steps; // Conversion factor from meters to steps
    const float steps_to_meters; // Conversion factor from steps to meters
};

#endif // NEMAMOTORS_HPP