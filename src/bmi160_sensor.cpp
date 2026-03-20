#include "../include/bmi160_sensor.hpp"
#include <rmw_microros/rmw_microros.h>

// Pins
#define IMU_ADDRESS 0x69
#define SDA_PIN 13
#define SCL_PIN 14 // Address and pins

Bmi160Sensor::Bmi160Sensor() {}

void Bmi160Sensor::init()
{
    Wire.begin(SDA_PIN, SCL_PIN);

    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0)
    {
        Serial.println("ERROR: IMU not detected");
    }
    else
    {
        Serial.println("IMU ok ...");
    }

    // Alojar memoria para TF array, igual que hacen los NemaMotors
    tf_msg.transforms.capacity = 1;
    tf_msg.transforms.size = 1;
    tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *)malloc(sizeof(geometry_msgs__msg__TransformStamped));
    memset(tf_msg.transforms.data, 0, sizeof(geometry_msgs__msg__TransformStamped));
}

void Bmi160Sensor::timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL) return;

    AccelData accelData;
    GyroData gyroData;

    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    // ---------------------------------------------------------
    // 1. TIEMPO Y DT
    // ---------------------------------------------------------
    int64_t current_time_ms = ros_synced_time_ms + (millis() - ms_before_sync);
    static int64_t previous_time_ms = current_time_ms;
    
    float dt = (current_time_ms - previous_time_ms) / 1000.0f;
    previous_time_ms = current_time_ms;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;

    // ---------------------------------------------------------
    // 2. VARIABLES DE ESTADO ESTÁTICAS Y CALIBRACIÓN
    // ---------------------------------------------------------
    static float x = 0.0f, y = 0.0f, theta = 0.0f;
    static float vx = 0.0f, vy = 0.0f;
    
    // Variables para calcular el Bias (Error de fábrica/inclinación)
    static float bias_ax = 0.0f, bias_ay = 0.0f, bias_wz = 0.0f;
    static int calib_count = 0;
    const int CALIB_SAMPLES = 50; // Calibra durante aprox 2.5 segundos

    float deg_to_rad = PI / 180.0f;
    float ax_raw = accelData.accelX * 9.80665f;
    float ay_raw = accelData.accelY * 9.80665f;
    float az_raw = accelData.accelZ * 9.80665f;
    float wz_raw = gyroData.gyroZ * deg_to_rad;

    // Proceso de calibración inicial (El robot DEBE estar quieto)
    if (calib_count < CALIB_SAMPLES) {
        bias_ax += ax_raw;
        bias_ay += ay_raw;
        bias_wz += wz_raw;
        calib_count++;
        
        if (calib_count == CALIB_SAMPLES) {
            bias_ax /= CALIB_SAMPLES;
            bias_ay /= CALIB_SAMPLES;
            bias_wz /= CALIB_SAMPLES;
            Serial.println("[IMU] Calibracion completada. Iniciando odometria visual.");
        }
        return; // Salimos de la función sin publicar hasta estar calibrados
    }

    // ---------------------------------------------------------
    // 3. SEÑALES FILTRADAS
    // ---------------------------------------------------------
    float ax_local = ax_raw - bias_ax;
    float ay_local = ay_raw - bias_ay;
    float wz_local = wz_raw - bias_wz;

    // ---------------------------------------------------------
    // 4. PUBLICAR TOPIC IMU_RAW
    // ---------------------------------------------------------
    imu_msg.header.frame_id.data = "imu_frame";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.stamp.sec = current_time_ms / 1000;
    imu_msg.header.stamp.nanosec = (current_time_ms % 1000) * 1000000;
    
    imu_msg.linear_acceleration.x = ax_raw; // ROS siempre espera el raw con gravedad en Z
    imu_msg.linear_acceleration.y = ay_raw;
    imu_msg.linear_acceleration.z = az_raw;
    imu_msg.angular_velocity.x = gyroData.gyroX * deg_to_rad;
    imu_msg.angular_velocity.y = gyroData.gyroY * deg_to_rad;
    imu_msg.angular_velocity.z = wz_raw;

    if (imu_pub.impl != NULL) rcl_publish(&imu_pub, &imu_msg, NULL);

    // ---------------------------------------------------------
    // 5. ODOMETRÍA POR IMU (ZUPT + Integración)
    // ---------------------------------------------------------
    
    // Rotación
    if (abs(wz_local) > 0.03f) { // Deadband para giro
        theta += wz_local * dt;
    }
    if (theta > PI) theta -= 2 * PI;
    if (theta < -PI) theta += 2 * PI;

    // Filtro ZUPT: Si la aceleración sin bias es menor a 0.3 m/s^2, asumimos que es ruido
    if (abs(ax_local) < 0.3f) ax_local = 0.0f;
    if (abs(ay_local) < 0.3f) ay_local = 0.0f;

    // Rotar aceleración al marco global (odom)
    float ax_global = ax_local * cos(theta) - ay_local * sin(theta);
    float ay_global = ax_local * sin(theta) + ay_local * cos(theta);

    // Integrar aceleración para obtener velocidad
    vx += ax_global * dt;
    vy += ay_global * dt;

    // Fricción ZUPT extrema: Si no detectamos aceleración empujando, frenar rápidamente el drift
    if (ax_local == 0.0f && ay_local == 0.0f) {
        vx *= 0.5f; // Decaimiento súper agresivo
        vy *= 0.5f;
    } else {
        vx *= 0.98f; // Decaimiento suave por rozamiento natural
        vy *= 0.98f;
    }

    // Forzar el 0 absoluto si es microscópico
    if (abs(vx) < 0.01f) vx = 0.0f;
    if (abs(vy) < 0.01f) vy = 0.0f;

    // Integrar velocidad para obtener posición
    x += vx * dt;
    y += vy * dt;

    // ---------------------------------------------------------
    // 6. PUBLICAR TF
    // ---------------------------------------------------------
    if (tf_pub.impl != NULL && rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
    {
        tf_msg.transforms.data[0].header.stamp.sec = imu_msg.header.stamp.sec;
        tf_msg.transforms.data[0].header.stamp.nanosec = imu_msg.header.stamp.nanosec;
        tf_msg.transforms.data[0].header.frame_id.data = "odom";
        tf_msg.transforms.data[0].header.frame_id.size = strlen(tf_msg.transforms.data[0].header.frame_id.data);
        tf_msg.transforms.data[0].child_frame_id.data = "base_link";
        tf_msg.transforms.data[0].child_frame_id.size = strlen(tf_msg.transforms.data[0].child_frame_id.data);
        
        tf_msg.transforms.data[0].transform.translation.x = x;
        tf_msg.transforms.data[0].transform.translation.y = y;
        tf_msg.transforms.data[0].transform.translation.z = 0.0;
        
        tf_msg.transforms.data[0].transform.rotation.x = 0.0;
        tf_msg.transforms.data[0].transform.rotation.y = 0.0;
        tf_msg.transforms.data[0].transform.rotation.z = sin(theta / 2.0);
        tf_msg.transforms.data[0].transform.rotation.w = cos(theta / 2.0);

        rcl_publish(&tf_pub, &tf_msg, NULL);
    }
}