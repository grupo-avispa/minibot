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

// =============================================================================
//  PIDController
//  Controlador PID discreto con:
//    · Anti-windup por clamp de integral
//    · Reset limpio en parada y cambio de signo
//    · Filtro paso-bajo en la derivada (DERIV_ALPHA)
// =============================================================================
struct PIDController {

    float kp, ki, kd;
    float integral;
    float prev_error;
    float prev_deriv;
    float output_min;
    float output_max;
    float integral_limit;

    // Suavizado de la derivada: 0 = sin filtro, valores altos = más suave
    static constexpr float DERIV_ALPHA = 0.5f;

    PIDController(float kp, float ki, float kd,
                  float out_min, float out_max, float int_lim)
        : kp(kp), ki(ki), kd(kd),
          integral(0.0f), prev_error(0.0f), prev_deriv(0.0f),
          output_min(out_min), output_max(out_max),
          integral_limit(int_lim) {}

    // Calcula salida PID. dt en segundos.
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;

        const float error = setpoint - measured;

        // Integral con anti-windup
        integral += error * dt;
        integral  = constrain(integral, -integral_limit, integral_limit);

        // Derivada con filtro paso-bajo
        const float raw_deriv  = (error - prev_error) / dt;
        const float filt_deriv = DERIV_ALPHA * prev_deriv + (1.0f - DERIV_ALPHA) * raw_deriv;
        prev_deriv = filt_deriv;
        prev_error = error;

        const float out = kp * error + ki * integral + kd * filt_deriv;
        return constrain(out, output_min, output_max);
    }

    void reset() {
        integral   = 0.0f;
        prev_error = 0.0f;
        prev_deriv = 0.0f;
    }
};

// =============================================================================
//  JgaMotors
// =============================================================================
class JgaMotors {
public:
    JgaMotors();
    ~JgaMotors();

    void init();
    void emergency_stop();
    void cmd_vel_sub_callback(const void *msgin);
    void timer_odometry_callback(rcl_timer_t *timer, int64_t last_call_time);
    void update_pose();

    // ── ROS ──────────────────────────────────────────────────────────────────
    rcl_publisher_t    odom_pub;
    rcl_publisher_t    tf_pub;
    rcl_subscription_t cmd_vel_sub;
    rcl_timer_t        timer_odom;

    nav_msgs__msg__Odometry    odom_msg;
    tf2_msgs__msg__TFMessage   tf_msg;
    geometry_msgs__msg__Twist  cmd_vel_msg;

    // ── Sincronización de tiempo micro-ROS ────────────────────────────────────
    int64_t ros_synced_time_ms = 0;
    int64_t ros_synced_time_ns = 0;
    long    ms_before_sync     = 0;

private:
    // ── Métodos privados ──────────────────────────────────────────────────────
    int  pwmFromPid(float pid_output);
    void applyMotorPwm(int pwm_l, int pwm_r);
    void updateRamp(float dt);
    void checkFullTurns(int64_t d_l, int64_t d_r);

    // ─────────────────────────────────────────────────────────────────────────
    //  PINES
    // ─────────────────────────────────────────────────────────────────────────
    static const int IN1_L   = 21;
    static const int IN2_L   = 19;
    static const int ENC_A_L = 32;
    static const int ENC_B_L = 33;

    static const int IN1_R   = 4;
    static const int IN2_R   = 27;
    static const int ENC_A_R = 34;   // GPIO input-only: válido para encoder
    static const int ENC_B_R = 35;   // GPIO input-only: válido para encoder

    // LEDs de diagnóstico de vueltas
    static const int LED_RED  = 16;   // conmuta en cada vuelta completa izquierda
    static const int LED_BLUE = 17;   // conmuta en cada vuelta completa derecha

    // PWM mínimo real para vencer la fricción estática
    static const int MIN_PWM_REAL = 40;

    // ─────────────────────────────────────────────────────────────────────────
    //  HARDWARE
    // ─────────────────────────────────────────────────────────────────────────
    MX1508      *motor_l = nullptr;
    MX1508      *motor_r = nullptr;
    ESP32Encoder encoder_l;
    ESP32Encoder encoder_r;

    // ─────────────────────────────────────────────────────────────────────────
    //  CONSTANTES MECÁNICAS
    // ─────────────────────────────────────────────────────────────────────────
    static constexpr float RADIO_RUEDA_M          = 0.011f;
    static constexpr float WHEEL_BASE             = 0.21f;
    static constexpr float TICKS_POR_VUELTA_TOTAL = 44.0f * 45.0f;        // 1980
    static constexpr float PERIMETRO_RUEDA_M      = 2.0f * 3.14159265f * RADIO_RUEDA_M;
    static constexpr float METROS_POR_TICK        = PERIMETRO_RUEDA_M / TICKS_POR_VUELTA_TOTAL;
    static constexpr float MAX_VEL_M_S            = (130.0f / 60.0f) * PERIMETRO_RUEDA_M;
    static constexpr float RPM_POR_M_S            = 60.0f / PERIMETRO_RUEDA_M;

    // ─────────────────────────────────────────────────────────────────────────
    //  PID
    //  Kp=1300 / Ki=4000 / Kd=0 tomados del .ino de referencia.
    //  Afina con pid_tuner.cpp antes de flashear a producción.
    // ─────────────────────────────────────────────────────────────────────────
    PIDController pid_l{ 1300.0f, 3500.0f, 10.0f, -255.0f, 255.0f, 200.0f };
    PIDController pid_r{ 1300.0f, 3500.0f, 10.0f, -255.0f, 255.0f, 200.0f };

    // ─────────────────────────────────────────────────────────────────────────
    //  RAMPA DE SETPOINT
    //  Limita la tasa de cambio en m/s². Evita el golpe brusco al pasar de
    //  velocidades muy dispares (p. ej. -0.10 → +0.05 m/s).
    // ─────────────────────────────────────────────────────────────────────────
    static constexpr float RAMP_RATE_M_S2 = 0.5f;

    float goal_vel_l   = 0.0f;   // objetivo final de cmd_vel
    float goal_vel_r   = 0.0f;
    float ramped_vel_l = 0.0f;   // setpoint suavizado → entrada del PID
    float ramped_vel_r = 0.0f;

    // ─────────────────────────────────────────────────────────────────────────
    //  FILTRO PASO-BAJO VELOCIDAD MEDIDA
    //  Igual que el .ino de referencia (ALPHA = 0.4).
    // ─────────────────────────────────────────────────────────────────────────
    static constexpr float VEL_ALPHA = 0.4f;
    float filtered_vel_l = 0.0f;
    float filtered_vel_r = 0.0f;

    // ─────────────────────────────────────────────────────────────────────────
    //  ARRANQUE SUAVE
    //  Aplica N ciclos de PWM mínimo directo al arrancar desde parado o al
    //  invertir el sentido, antes de ceder el control al PID.
    // ─────────────────────────────────────────────────────────────────────────
    static const int ARRANQUE_CICLOS = 3;
    int arranque_ciclos_l = 0;
    int arranque_ciclos_r = 0;
    int signo_anterior_l  = 0;
    int signo_anterior_r  = 0;

    // ─────────────────────────────────────────────────────────────────────────
    //  BANDA MUERTA Y ANCLA
    // ─────────────────────────────────────────────────────────────────────────
    static constexpr float BANDA_MUERTA_M_S = 0.005f;
    bool ancla = false;   // true → motorBrake activo con setpoint=0

    // ─────────────────────────────────────────────────────────────────────────
    //  POSE ODOMÉTRICA
    // ─────────────────────────────────────────────────────────────────────────
    float   x     = 0.0f;
    float   y     = 0.0f;
    float   theta = 0.0f;

    // ─────────────────────────────────────────────────────────────────────────
    //  ESTADO ENCODERS / TIEMPO
    // ─────────────────────────────────────────────────────────────────────────
    int64_t prev_ticks_left  = 0;
    int64_t prev_ticks_right = 0;
    int64_t prev_time_ms     = 0;

    // ─────────────────────────────────────────────────────────────────────────
    //  CONTADORES DE VUELTAS Y ESTADO LEDs
    // ─────────────────────────────────────────────────────────────────────────
    int64_t full_turns_left   = 0;
    int64_t full_turns_right  = 0;
    int64_t total_ticks_left  = 0;
    int64_t total_ticks_right = 0;
    bool    led_red_state     = false;
    bool    led_blue_state    = false;
};

#endif // JGAMOTORS_HPP