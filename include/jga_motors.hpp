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

// --- MÁQUINA DE ESTADOS ---
enum EstadoSistema {
    ESTADO_NORMAL,   // PID trabajando con normalidad
    ESTADO_PARANDO,  // Bajando velocidad hasta VEL_MIN_MS antes de frenar
    ESTADO_FRENO,    // Poniendo PWM a 0 y reseteando PID
    ESTADO_STOPPED   // Parado en seco, esperando comando
};

// --- IMPLEMENTACIÓN DEL PID (.ino) ---
struct CustomPID {
    float kp, ki, kd;
    float e_n1, e_n2, r_n1;
    float outMin, outMax;

    CustomPID(float p, float i, float d) {
        kp = p; ki = i; kd = d;
        e_n1 = 0.0f; e_n2 = 0.0f; r_n1 = 0.0f;
        outMin = -255.0f; outMax = 255.0f;
    }

    void setTunings(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }

    void reset() {
        e_n1 = 0.0f; e_n2 = 0.0f; r_n1 = 0.0f;
    }

    float compute(float setpoint, float input) {
        float e_n = setpoint - input;
        float r_n = r_n1 + kp * (e_n - e_n1) + ki * (e_n + e_n1) / 2.0f + kd * (e_n - 2.0f * e_n1 + e_n2);
        
        if (r_n > outMax) r_n = outMax;
        if (r_n < outMin) r_n = outMin;

        e_n2 = e_n1;
        e_n1 = e_n;
        r_n1 = r_n;

        return r_n;
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
    void update_pose(long deltaTicks_l, long deltaTicks_r);

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
    float msToRPM(float m_s);
    int aplicarZonaMuerta(int pwm);
    void resetPID();
    void arrancarMotor(float sp_l, float sp_r);

    // ─────────────────────────────────────────────────────────────────────────
    //  PINES
    // ─────────────────────────────────────────────────────────────────────────
    static const int IN1_L   = 21;
    static const int IN2_L   = 19;
    static const int ENC_A_L = 32;
    static const int ENC_B_L = 33;

    static const int IN1_R   = 4;
    static const int IN2_R   = 27;
    static const int ENC_A_R = 34;   
    static const int ENC_B_R = 35;   

    static const int LED_RED  = 16;
    static const int LED_BLUE = 17;

    // ─────────────────────────────────────────────────────────────────────────
    //  CONFIGURACIÓN FÍSICA Y LÍMITES
    // ─────────────────────────────────────────────────────────────────────────
    static const int MIN_PWM_REAL = 40;
    static constexpr float TICKS_POR_VUELTA = 1980.0f;  
    static constexpr float RADIO_RUEDA_M = 0.0425f;  
    static constexpr float PERIMETRO_RUEDA_M = 2.0f * PI * RADIO_RUEDA_M;
    static constexpr float WHEEL_BASE = 0.21f;

    float VEL_MIN_MS = 0.15f;
    float VEL_MAX_MS = 0.50f;

    static const int PERIODO_MS = 50;
    static constexpr float ALPHA = 0.4f;
    static constexpr float RAMPA_CAMBIO_RPM = 10.0f;
    static constexpr float BANDA_MUERTA_RPM = 2.0f;
    static const int CICLOS_ARRANQUE = 3;

    // ─────────────────────────────────────────────────────────────────────────
    //  HARDWARE & PID
    // ─────────────────────────────────────────────────────────────────────────
    MX1508      *motor_l = nullptr;
    MX1508      *motor_r = nullptr;
    ESP32Encoder encoder_l;
    ESP32Encoder encoder_r;

    float global_kp = 0.8f;
    float global_ki = 0.4f;
    float global_kd = 0.1f;

    CustomPID pid_l{global_kp, global_ki, global_kd};
    CustomPID pid_r{global_kp, global_ki, global_kd};

    // ─────────────────────────────────────────────────────────────────────────
    //  ESTADO DEL SISTEMA (VARIABLES DEL .ino)
    // ─────────────────────────────────────────────────────────────────────────
    EstadoSistema estadoActual = ESTADO_STOPPED;

    float rpmFiltrado_l = 0.0f;
    float rpmFiltrado_r = 0.0f;

    long posAnterior_l = 0;
    long posAnterior_r = 0;

    float setpointRampaRPM_l    = 0.0f;
    float setpointRampaRPM_r    = 0.0f;
    float setpointObjetivoRPM_l = 0.0f;
    float setpointObjetivoRPM_r = 0.0f;
    
    unsigned long tAnterior = 0;

    int signoAnterior_l = 0;
    int signoAnterior_r = 0;
    int ciclosArranque_l = 0;
    int ciclosArranque_r = 0;

    // ─────────────────────────────────────────────────────────────────────────
    //  POSE ODOMÉTRICA
    // ─────────────────────────────────────────────────────────────────────────
    float x     = 0.0f;
    float y     = 0.0f;
    float theta = 0.0f;
};

#endif // JGAMOTORS_HPP