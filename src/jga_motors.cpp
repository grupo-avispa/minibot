#include "../include/jga_motors.hpp"
#include <rmw_microros/rmw_microros.h>

JgaMotors::JgaMotors()
    : x(0.0f), y(0.0f), theta(0.0f),
      prev_ticks_left(0), prev_ticks_right(0)
{
    // No se crean los objetos MX1508 aquí.
    // En C++ los objetos globales se construyen antes de setup().
    // El framework Arduino no ha configurado los pines todavía en ese momento,
    // por lo que el MX1508 registraría los canales LEDC incorrectamente.
    // Se crean en init() que se llama desde setup().
}

JgaMotors::~JgaMotors()
{
    delete motor_l;
    delete motor_r;
}

void JgaMotors::init()
{
    // 1. Crear motores AQUÍ, dentro de setup(), cuando Arduino ya está listo
    motor_l = new MX1508(IN1_L, IN2_L, SLOW_DECAY);
    motor_r = new MX1508(IN1_R, IN2_R, SLOW_DECAY);

    Serial.printf("[JGA] Motores creados: L(IN1=%d IN2=%d) R(IN1=%d IN2=%d)\n",
                  IN1_L, IN2_L, IN1_R, IN2_R);

    // 2. Encoders
    pinMode(ENC_A_L, INPUT_PULLUP);
    pinMode(ENC_B_L, INPUT_PULLUP);
    pinMode(ENC_A_R, INPUT_PULLUP);
    pinMode(ENC_B_R, INPUT_PULLUP);

    encoder_l.attachFullQuad(ENC_A_L, ENC_B_L);
    encoder_r.attachFullQuad(ENC_A_R, ENC_B_R);
    encoder_l.clearCount();
    encoder_r.clearCount();
    encoder_l.setFilter(1023);
    encoder_r.setFilter(1023);

    // 3. Estado inicial seguro
    motor_l->motorStop();
    motor_r->motorStop();

    // 4. Reserva de memoria TF ROS
    tf_msg.transforms.capacity = 1;
    tf_msg.transforms.size = 1;
    tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *)malloc(
        sizeof(geometry_msgs__msg__TransformStamped));
    memset(tf_msg.transforms.data, 0, sizeof(geometry_msgs__msg__TransformStamped));

    Serial.printf("[JGA] MAX_VEL=%.4f m/s  MIN_PWM=%d  M/TICK=%.6f\n",
                  MAX_VEL_M_S, MIN_PWM_REAL, METROS_POR_TICK);
    Serial.println("[JGA] Init OK");
}

void JgaMotors::emergency_stop()
{
    if (motor_l) motor_l->motorStop();
    if (motor_r) motor_r->motorStop();
    Serial.println("[JGA] Emergency stop.");
}

int JgaMotors::aplicarZonaMuerta(float vel_m_s)
{
    if (fabsf(vel_m_s) < 0.001f) return 0;

    int dir   = (vel_m_s > 0) ? 1 : -1;
    float ratio = fabsf(vel_m_s) / MAX_VEL_M_S;
    if (ratio > 1.0f) ratio = 1.0f;

    int vel_abs = (int)(ratio * 255.0f);
    if (vel_abs < 1) vel_abs = 1;

    int pwm = (int)map((long)vel_abs, 1, 255, MIN_PWM_REAL, 255);
    return pwm * dir;
}

void JgaMotors::cmd_vel_sub_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float lin = msg->linear.x;
    float ang = msg->angular.z;

    float v_l = lin - (ang * WHEEL_BASE / 2.0f);
    float v_r = lin + (ang * WHEEL_BASE / 2.0f);

    int pwm_l = aplicarZonaMuerta(v_l);
    int pwm_r = aplicarZonaMuerta(v_r);

    Serial.printf("[CMD] lin=%.3f ang=%.3f | vL=%.3f vR=%.3f | pwmL=%d pwmR=%d\n",
                  lin, ang, v_l, v_r, pwm_l, pwm_r);

    if (pwm_l == 0) motor_l->motorStop();
    else            motor_l->motorGo(pwm_l);

    // Motor derecho invertido por montaje en espejo
    if (pwm_r == 0) motor_r->motorStop();
    else            motor_r->motorGo(-pwm_r);
}

void JgaMotors::update_pose()
{
    int64_t cur_l =  encoder_l.getCount();
    int64_t cur_r = -encoder_r.getCount();

    int64_t d_l = cur_l - prev_ticks_left;
    int64_t d_r = cur_r - prev_ticks_right;

    prev_ticks_left  = cur_l;
    prev_ticks_right = cur_r;

    float dL = (float)d_l * METROS_POR_TICK;
    float dR = (float)d_r * METROS_POR_TICK;
    float dC = (dR + dL) / 2.0f;
    float dTheta = (dR - dL) / WHEEL_BASE;

    x     += dC * cosf(theta + dTheta / 2.0f);
    y     += dC * sinf(theta + dTheta / 2.0f);
    theta += dTheta;

    if (theta >  PI) theta -= 2.0f * PI;
    if (theta < -PI) theta += 2.0f * PI;
}

void JgaMotors::timer_odometry_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL) return;

    update_pose();

    int64_t t_ms = ros_synced_time_ms + (millis() - ms_before_sync);

    odom_msg.header.stamp.sec     = (int32_t)(t_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((t_ms % 1000) * 1000000);
    odom_msg.header.frame_id.data = const_cast<char*>("odom");
    odom_msg.header.frame_id.size = 4;
    odom_msg.child_frame_id.data  = const_cast<char*>("base_link");
    odom_msg.child_frame_id.size  = 9;

    odom_msg.pose.pose.position.x    = x;
    odom_msg.pose.pose.position.y    = y;
    odom_msg.pose.pose.position.z    = 0.0f;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);

    if (odom_pub.impl != NULL)
        rcl_publish(&odom_pub, &odom_msg, NULL);

    tf_msg.transforms.data[0].header.stamp.sec     = odom_msg.header.stamp.sec;
    tf_msg.transforms.data[0].header.stamp.nanosec = odom_msg.header.stamp.nanosec;
    tf_msg.transforms.data[0].header.frame_id.data = const_cast<char*>("odom");
    tf_msg.transforms.data[0].header.frame_id.size = 4;
    tf_msg.transforms.data[0].child_frame_id.data  = const_cast<char*>("base_link");
    tf_msg.transforms.data[0].child_frame_id.size  = 9;
    tf_msg.transforms.data[0].transform.translation.x = x;
    tf_msg.transforms.data[0].transform.translation.y = y;
    tf_msg.transforms.data[0].transform.translation.z = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.z    = odom_msg.pose.pose.orientation.z;
    tf_msg.transforms.data[0].transform.rotation.w    = odom_msg.pose.pose.orientation.w;

    if (tf_pub.impl != NULL)
        rcl_publish(&tf_pub, &tf_msg, NULL);
}