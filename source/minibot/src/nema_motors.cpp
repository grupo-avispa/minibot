/*
 * Minibot — autonomous differential-drive robot for ROS 2 teaching
 *
 * Copyright (c) 2026 Óscar Pons Fernández
 * Copyright (c) 2026 José Galeas
 * Copyright (c) 2026 Alberto J. Tudela Roldán
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
#include "nema_motors.hpp"
#include <cmath>      // For M_PI
#include <rmw_microros/rmw_microros.h>

// Motor control configuration
#define STEP_PIN_L   19
#define DIR_PIN_L    21
#define STEP_PIN_R   32
#define DIR_PIN_R    33
#define MOTOR_ENABLE 23 
#define MICRO_STEP   2       // Microstepping setting (hardware)
#define MAX_VEL      600     // Maximum velocity in steps/s

// Constructor
NemaMotors::NemaMotors()
    : step_pin_l(STEP_PIN_L),
    dir_pin_l(DIR_PIN_L),
    step_pin_r(STEP_PIN_R),
    dir_pin_r(DIR_PIN_R),
    motor_enable_pin(MOTOR_ENABLE),
    micro_step(MICRO_STEP),
    max_vel(MAX_VEL),
    stepper_l(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L),
    stepper_r(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R),
    wheel_radius(0.043f),          // 4.3 cm wheel radius
    wheel_base(0.21f),            // 21 cm between wheels
    x(0.0f),
    y(0.0f),
    theta(0.0f),
    prev_steps_left(0),
    prev_steps_right(0),
    steps_per_rev(200.0f * MICRO_STEP),
    rad_per_step((2.0f * M_PI) / (200.0f * MICRO_STEP)),
    meters_to_steps(1.0f / (0.05f * ((2.0f * M_PI) / (200.0f * MICRO_STEP)))),
    steps_to_meters(0.05f * ((2.0f * M_PI) / (200.0f * MICRO_STEP))){

    // Serial used to send data to PC
    Serial.begin(115200);
    
    // Enable lidar motor 
    pinMode(motor_enable_pin, OUTPUT);
    digitalWrite(motor_enable_pin, LOW); // Set pin 23 low to enable the motor
}

// Destructor
NemaMotors::~NemaMotors() {
    // Cleanup if needed (usually nothing for AccelStepper)
}


void NemaMotors::init() {
    // Initialize max speed for both motors
    stepper_l.setMaxSpeed(max_vel);
    stepper_r.setMaxSpeed(max_vel);
    // Set initial speed to 0
    stepper_l.setSpeed(0);
    stepper_r.setSpeed(0);
    // Set current position to 0
    stepper_l.setCurrentPosition(0);
    stepper_r.setCurrentPosition(0);

    // Allocate memory for TF array (only one transform in this case)
    tf_msg.transforms.capacity = 1;
    tf_msg.transforms.size = 1;
    tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *)malloc(sizeof(geometry_msgs__msg__TransformStamped));
    memset(tf_msg.transforms.data, 0, sizeof(geometry_msgs__msg__TransformStamped));
}

void NemaMotors::run_motors() {
    // This function is called every 500 us
    // It runs the stepper motors at the latest speed set by the cmd_vel subscription callback
    stepper_l.runSpeed();
    stepper_r.runSpeed();
}

void NemaMotors::cmd_vel_sub_callback(const void *msgin) {
    // Serial.print("Received cmd_vel...\n");
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear_velocity = msg->linear.x;  // m/s
    float angular_velocity = msg->angular.z; // rad/s

    Serial.printf("Linear velocity: %.2f m/s, Angular velocity: %.2f rad/s\n", linear_velocity, angular_velocity);

    // Calculate stepper speed in steps for differential drive wheel_base
    float left_step_speed = ((linear_velocity / wheel_radius) - 0.5 * wheel_base * (angular_velocity / wheel_radius)) / rad_per_step; 
    float right_step_speed = ((linear_velocity / wheel_radius) + 0.5 * wheel_base * (angular_velocity / wheel_radius)) / rad_per_step;

    // Limit the speed to max_vel
    if (left_step_speed > max_vel) {
        left_step_speed = max_vel;
    } else if (left_step_speed < -max_vel) {
        left_step_speed = -max_vel;
    }
    if (right_step_speed > max_vel) {
        right_step_speed = max_vel;
    } else if (right_step_speed < -max_vel) {
        right_step_speed = -max_vel;
    }
    // Set the speed for both motors
    stepper_l.setSpeed(left_step_speed);
    stepper_r.setSpeed(right_step_speed);

    Serial.printf("Left step speed: %.2f steps/s, Right step speed: %.2f steps/s\n", left_step_speed, right_step_speed);
}

void NemaMotors::update_pose(const long *steps_left, const long *steps_right) {
    // Calculate delta steps since last update
    long delta_steps_left = *steps_left - prev_steps_left;
    long delta_steps_right = *steps_right - prev_steps_right;

    // Store previous steps for next iteration
    prev_steps_left = *steps_left;
    prev_steps_right = *steps_right;

    // Steps to meters conversion
    float dL = delta_steps_left * steps_to_meters; // Distance traveled by the left wheel
    float dR = delta_steps_right * steps_to_meters; // Distance traveled by the right wheel

    // robot odometry calculations
    float dC = (dR + dL) / 2.0;
    float dTheta = (dR - dL) / wheel_base;

    // Update robot position
    float delta_x = dC * cos(theta + dTheta / 2.0);
    float delta_y = dC * sin(theta + dTheta / 2.0);

    x += delta_x;
    y += delta_y;
    theta += dTheta;

    // Normalize theta to be within -PI to PI
    if (theta > PI) theta -= 2 * PI;
    if (theta < -PI) theta += 2 * PI;
}

void NemaMotors::timer_odometry_callback(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer != NULL) {
        // Get current time
        odom_msg.header.stamp.sec = (ros_synced_time_ms + millis() - ms_before_sync) / 1000;
        odom_msg.header.stamp.nanosec = ros_synced_time_ms + (micros() * 1000) % 1000000000;
        // Compute current position based on stepper motor positions
        long stepper_l_pos = stepper_l.currentPosition();
        long stepper_r_pos = stepper_r.currentPosition();
        update_pose(&stepper_l_pos, &stepper_r_pos);
        // Fill odometry message
        odom_msg.header.frame_id.data = "odom";
        odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
        odom_msg.child_frame_id.data = "base_link";
        odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
        odom_msg.pose.pose.position.x = x;  // m
        odom_msg.pose.pose.position.y = y;  // m
        odom_msg.pose.pose.position.z = 0.0; // m
        odom_msg.pose.pose.orientation.x = 0.0; // Quaternion representation of orientation
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(theta / 2.0); // z component of quaternion
        odom_msg.pose.pose.orientation.w = cos(theta / 2.0); // w component of quaternion

        // Publish the odometry message
        if (odom_pub.impl == NULL || rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            // If the agent is not reachable, do not publish
            Serial.println("Agent not reachable, skipping odometry publish.");
            return;
        }
        rcl_publish(&odom_pub, &odom_msg, NULL);

        // Fill the TF message
        tf_msg.transforms.data[0].header.stamp.sec = odom_msg.header.stamp.sec;
        tf_msg.transforms.data[0].header.stamp.nanosec = odom_msg.header.stamp.nanosec;
        tf_msg.transforms.data[0].header.frame_id.data = "odom";
        tf_msg.transforms.data[0].header.frame_id.size = strlen(tf_msg.transforms.data[0].header.frame_id.data);
        tf_msg.transforms.data[0].child_frame_id.data = "base_link";
        tf_msg.transforms.data[0].child_frame_id.size = strlen(tf_msg.transforms.data[0].child_frame_id.data);
        tf_msg.transforms.data[0].transform.translation.x = x;  // m
        tf_msg.transforms.data[0].transform.translation.y = y;  // m
        tf_msg.transforms.data[0].transform.translation.z = 0.0; // m
        tf_msg.transforms.data[0].transform.rotation.x = 0.0; // Quaternion representation of orientation
        tf_msg.transforms.data[0].transform.rotation.y = 0.0;
        tf_msg.transforms.data[0].transform.rotation.z = odom_msg.pose.pose.orientation.z; // z component of quaternion
        tf_msg.transforms.data[0].transform.rotation.w = odom_msg.pose.pose.orientation.w; // w component of quaternion

        // Publish the TF message
        if (tf_pub.impl == NULL || rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            // If the agent is not reachable, do not publish
            Serial.println("Agent not reachable, skipping TF publish.");
            return;
        }
        rcl_publish(&tf_pub, &tf_msg, NULL);
    }
}