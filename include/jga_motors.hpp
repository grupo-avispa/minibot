#ifndef JGAMOTORS_HPP
#define JGAMOTORS_HPP

#include <Arduino.h>
#include <FS_MX1508.h>
#include <ESP32Encoder.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>

class JgaMotors {
public:
    JgaMotors();
    ~JgaMotors();

    void init();
    void emergency_stop();
    void cmd_vel_sub_callback(const void *msgin);
    void timer_odometry_callback(rcl_timer_t *timer, int64_t last_call_time);
    void update_pose();

    rcl_publisher_t    odom_pub;
    rcl_publisher_t    tf_pub;
    rcl_subscription_t cmd_vel_sub;
    rcl_timer_t        timer_odom;

    nav_msgs__msg__Odometry    odom_msg;
    tf2_msgs__msg__TFMessage   tf_msg;
    geometry_msgs__msg__Twist  cmd_vel_msg;

    int64_t ros_synced_time_ms = 0;
    int64_t ros_synced_time_ns = 0;
    long    ms_before_sync     = 0;

private:
    int aplicarZonaMuerta(float vel_m_s);

    // Pines — definidos como constantes simples (no miembros const para evitar
    // problemas de inicialización antes de setup())
    static const int IN1_L   = 21;
    static const int IN2_L   = 19;
    static const int ENC_A_L = 32;
    static const int ENC_B_L = 33;

    static const int IN1_R   = 4;
    static const int IN2_R   = 27;
    static const int ENC_A_R = 34;  // input-only: OK para encoder
    static const int ENC_B_R = 35;  // input-only: OK para encoder

    static const int MIN_PWM_REAL = 40;

    // Motores como punteros — se crean en init() dentro de setup(),
    // cuando el framework Arduino ya tiene los pines configurados.
    MX1508      *motor_l = nullptr;
    MX1508      *motor_r = nullptr;
    ESP32Encoder encoder_l;
    ESP32Encoder encoder_r;

    static constexpr float RADIO_RUEDA_M        = 0.011f;
    static constexpr float WHEEL_BASE           = 0.21f;
    static constexpr float TICKS_POR_VUELTA_TOTAL = 44.0f * 45.0f;
    static constexpr float PERIMETRO_RUEDA_M    = 2.0f * 3.14159265f * RADIO_RUEDA_M;
    static constexpr float METROS_POR_TICK      = PERIMETRO_RUEDA_M / TICKS_POR_VUELTA_TOTAL;
    static constexpr float MAX_VEL_M_S          = (130.0f / 60.0f) * PERIMETRO_RUEDA_M;

    float   x = 0.0f;
    float   y = 0.0f;
    float   theta = 0.0f;
    int64_t prev_ticks_left  = 0;
    int64_t prev_ticks_right = 0;
};

#endif // JGAMOTORS_HPP