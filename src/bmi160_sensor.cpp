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
        Serial.println("ERROR: BMI160 no detectado con FastIMU.");
    }
    else
    {
        Serial.println("BMI160 inicializado correctamente (FastIMU).");
    }
}

void Bmi160Sensor::timer_imu_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        AccelData accelData;
        GyroData gyroData;

        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        // Lógica de tiempo unificada
        // IMU Frame
        imu_msg.header.frame_id.data = "imu_frame";
        imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

        imu_msg.orientation_covariance[0] = -1.0;
        int64_t current_time_ms = ros_synced_time_ms + (millis() - ms_before_sync);
        imu_msg.header.stamp.sec = current_time_ms / 1000;
        imu_msg.header.stamp.nanosec = (current_time_ms % 1000) * 1000000;

        // Conversión a m/s^2
        imu_msg.linear_acceleration.x = accelData.accelX * 9.80665f;
        imu_msg.linear_acceleration.y = accelData.accelY * 9.80665f;
        imu_msg.linear_acceleration.z = accelData.accelZ * 9.80665f;

        // Conversión a Rad/s (Ajusta los signos si el giro sigue invertido)
        float deg_to_rad = PI / 180.0f;
        imu_msg.angular_velocity.x = (gyroData.gyroX * deg_to_rad);
        imu_msg.angular_velocity.y = (gyroData.gyroY * deg_to_rad);
        imu_msg.angular_velocity.z = (gyroData.gyroZ * deg_to_rad);

        // Publicación directa (El ping ya se hace en vTaskMicroROS)
        if (imu_pub.impl != NULL)
        {
            rcl_publish(&imu_pub, &imu_msg, NULL);
        }
    }
}