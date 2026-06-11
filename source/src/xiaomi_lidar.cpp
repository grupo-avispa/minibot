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
#include "xiaomi_lidar.hpp"
#include <rmw_microros/rmw_microros.h>

#define XIAOMI_LIDAR_FRAME_HEADER 0xAA
#define XIAOMI_LIDAR_PROTOCOL_VERSION 0x01
#define XIAOMI_LIDAR_FRAME_TYPE 0x61
#define XIAOMI_LIDAR_SCAN_STEPS 16
#define XIAOMI_LIDAR_SAMPLES_PER_FRAME 17
#define XIAOMI_LIDAR_END_OF_SCAN 65535
#define XIAOMI_LIDAR_FRAME_INDEX_INIT 0
#define XIAOMI_LIDAR_RX_PIN 26
#define XIAOMI_LIDAR_MOTOR_ENABLE_PIN 25

// Constructor
XiaomiLidar::XiaomiLidar()
    : FRAME_HEADER(XIAOMI_LIDAR_FRAME_HEADER),
      PROTOCOL_VERSION(XIAOMI_LIDAR_PROTOCOL_VERSION),
      FRAME_TYPE(XIAOMI_LIDAR_FRAME_TYPE),
      SCAN_STEPS(XIAOMI_LIDAR_SCAN_STEPS),
      SAMPLES_PER_FRAME(XIAOMI_LIDAR_SAMPLES_PER_FRAME),
      rotation_speed_scale(0.05f * 60.0f),   // Conversion factor to RPM
      angle_scale(0.01f),                    // Conversion factor to degrees
      range_scale(0.25f * 0.001f),           // Conversion factor to meters
      end_of_scan(XIAOMI_LIDAR_END_OF_SCAN), // Reserved value to mark end of scan
      frame_index(XIAOMI_LIDAR_FRAME_INDEX_INIT),
      last_start_angle(0),                   // Initialize zero-crossing tracker
      lidar_rx_pin(XIAOMI_LIDAR_RX_PIN),
      motor_lidar_enable_pin(XIAOMI_LIDAR_MOTOR_ENABLE_PIN), // Pin to enable/disable lidar motor
      lidar_data_queue(nullptr),
      serialTaskHandle(nullptr),
      rosPubTaskHandle(nullptr)
{
    // Setup
    // Serial used to send data to PC
    Serial.begin(115200);
    // Serial2 used to communicate with the Lidar
    Serial2.begin(115200, SERIAL_8N1, lidar_rx_pin, -1); // Initialize serial port for Lidar

    // Create FreeRTOS queue for communication between tasks
    lidar_data_queue = xQueueCreate(600, sizeof(uint16_t));

    // Enable lidar motor
    pinMode(motor_lidar_enable_pin, OUTPUT);
    digitalWrite(motor_lidar_enable_pin, HIGH); // Set pin 25 high to enable the motor
}

// Destructor
XiaomiLidar::~XiaomiLidar()
{
    // Free dynamically allocated memory if any
    if (scan_msg.ranges.data != nullptr)
    {
        free(scan_msg.ranges.data);
        scan_msg.ranges.data = nullptr;
    }
}

// Task that reads serial data, builds lidar frames, and pushes them for processing
void XiaomiLidar::SerialReadTask(void *pvParameters)
{
    static int status = 0;
    static uint16_t checksum = 0;
    static XiaomiLidarFrame frame;
    /* A dataframe is formed by:
        + The header, which is 0xAA
        + The frame length, which is 2 bytes (MSB and LSB)
        + The protocol version, which is 1 byte
        + The frame type, which is 1 byte
        + The command word, which is 1 byte
        + The payload length, which is 2 bytes (MSB and LSB)
        + The payload data, which is variable length
        + The checksum, which is 2 bytes (MSB and LSB)
        + The checksum calculation does not include the last 2 bytes of the frame (the proper checksum bytes)
    */
    frame.parameters.reserve(65);
    bool current_frame_is_end_of_scan = false;
    while (true)
    {
        while (Serial2.available())
        {
            uint8_t by = Serial2.read();
            switch (status)
            {
            case 0: // Frame Header
                frame.frameHeader = by;
                if (frame.frameHeader == FRAME_HEADER)
                {
                    checksum = by;
                    status = 1;
                }
                break;
            case 1: // Frame Length MSB
                frame.frameLength = by << 8;
                checksum += by;
                status = 2;
                break;
            case 2: // Frame Length LSB
                frame.frameLength |= by;
                checksum += by;
                status = 3;
                break;
            case 3: // Protocol Version
                frame.protocolVersion = by;
                checksum += by;
                if (frame.protocolVersion == PROTOCOL_VERSION)
                {
                    status = 4;
                }
                else
                {
                    Serial.println("ERROR: Protocol Version");
                    status = 0;
                }
                break;
            case 4: // Frame Type
                frame.frameType = by;
                checksum += by;
                if (frame.frameType == FRAME_TYPE)
                {
                    status = 5;
                }
                else
                {
                    Serial.println("ERROR: Frame Type");
                    status = 0;
                }
                break;
            case 5: // Command Word
                frame.commandWord = by;
                checksum += by;
                status = 6;
                break;
            case 6: // Parameter Length MSB
                frame.parameterLength = by << 8;
                checksum += by;
                status = 7;
                break;
            case 7: // Parameter Length LSB
                frame.parameterLength |= by;
                checksum += by;
                frame.parameters.clear();
                status = 8;
                break;
            case 8: // Parameters
                frame.parameters.push_back(by);
                checksum += by;
                if (frame.parameters.size() == frame.parameterLength)
                {
                    status = 9;
                }
                break;
            case 9: // Checksum MSB
                frame.checksum = by << 8;
                status = 10;
                break;
            case 10: // Checksum LSB
                frame.checksum |= by;
                if (frame.checksum == checksum)
                {
                    current_frame_is_end_of_scan = ProcessLidarFrame(frame);
                }
                else
                {
                    Serial.println("ERROR: Checksum mismatch");
                }
                status = 0;
                if (current_frame_is_end_of_scan)
                {
                    vTaskDelay(10);
                }
                break;
            }
            if (status < 10 && status > 0)
            {
                checksum %= 0xFFFF;
            }
        }
        vTaskDelay(1);
    }
}

// Task that waits for processed scan data and publishes it to ROS
void XiaomiLidar::RosPublishTask(void *pvParameters)
{
    std::vector<uint16_t> samples_vector;
    samples_vector.reserve(400); // Reserve a bit more space just in case
    bool end_of_scan_received = false;
    uint16_t scan;
    
    // Initialize the scan message constants
    scan_msg.range_min = 0.05;
    scan_msg.range_max = 6.0;
    scan_msg.header.frame_id.data = "laser_link";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    
    while (true)
    {
        if (xQueueReceive(lidar_data_queue, &scan, portMAX_DELAY) == pdPASS)
        {
            if (uxQueueMessagesWaiting(lidar_data_queue) > 590)
            {
                Serial.printf("Queue size: %d\n", uxQueueMessagesWaiting(lidar_data_queue));
            }
            
            // Si llega el marcador, cerramos el paquete de datos
            if (scan == end_of_scan)
            {
                end_of_scan_received = true;
            }
            else
            {
                samples_vector.push_back(scan);
            }
        }
        
        // Check if we received the end of scan signal
        if (end_of_scan_received)
        {
            end_of_scan_received = false;
            
            // Prevent publishing empty or corrupted scans
            if (samples_vector.size() > 0) {
                create_scan_msg(samples_vector);
                
                // Publish the scan message
                if (laser_publisher.impl != NULL || rmw_uros_ping_agent(150, 1) == RMW_RET_OK)
                {
                    rcl_publish(&laser_publisher, &scan_msg, NULL);
                    Serial.printf("Published scan with %d samples\n", scan_msg.ranges.size);
                }
                else
                {
                    Serial.println("Agent not reachable, skipping scan publish.");
                }
            }
            samples_vector.clear(); // Clear the vector for the next scan
        }
    }
}

// Constructs a LaserScan message from distance samples and fills scan_msg
void XiaomiLidar::create_scan_msg(const std::vector<uint16_t> &samples)
{
    // Fijamos los ángulos al círculo completo físico del Lidar
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_max = 2.0f * M_PI; 
    
    // El incremento depende únicamente del número total de muestras en este giro de 360 grados
    size_t num_samples = samples.size();
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / static_cast<float>(num_samples);

    // Lógica de tiempo unificada
    int64_t current_time_ms = ros_synced_time_ms + (millis() - ms_before_sync);
    scan_msg.header.stamp.sec = current_time_ms / 1000;
    scan_msg.header.stamp.nanosec = (current_time_ms % 1000) * 1000000;

    // Liberamos y reasignamos memoria. Note que ahora es samples.size() completo,
    // porque ya no guardamos el start_angle al final del array.
    if (scan_msg.ranges.data != nullptr)
    {
        free(scan_msg.ranges.data);
    }
    scan_msg.ranges.data = (float *)malloc(num_samples * sizeof(float));
    scan_msg.ranges.capacity = num_samples;
    scan_msg.ranges.size = num_samples;
    
    for (size_t i = 0; i < num_samples; ++i)
    {
        // Mantenemos la inversión que tenías, si es necesaria para la orientación de tu URDF
        scan_msg.ranges.data[i] = static_cast<float>(samples[num_samples - 1 - i]) * range_scale;
    }
}

// Processes a Xiaomi Lidar frame: extracts distances, angles and pushes them to the queue
bool XiaomiLidar::ProcessLidarFrame(const XiaomiLidarFrame &frame)
{
    bool current_frame_is_end_of_scan = false;
    switch (frame.commandWord)
    {
    case 0xAE: // Device Health Information
    {
        float rpm = frame.parameters[0] * rotation_speed_scale;
        Serial.printf("Device Health Information: \n");
        Serial.printf("RPM: %f\n", rpm);
    }
    break;
    case 0xAD: // Scan Data
    {
        uint16_t start_angle = (frame.parameters[3] << 8) | frame.parameters[4];

        int sample_cnt = static_cast<int>((frame.parameterLength - 5) / 3);
        for (int i = 0; i < sample_cnt; i++)
        {
            uint16_t distance = (frame.parameters[5 + (i * 3) + 1] << 8) | frame.parameters[5 + (i * 3) + 2];
            
            // Any distance can be end of scan marker evasion
            if (distance == end_of_scan)
            {
                distance = end_of_scan - 1;
            }
            
            // Mandamos solo la distancia a la cola
            if (xQueueSend(lidar_data_queue, &distance, portMAX_DELAY) != pdTRUE)
            {
                Serial.println("ERROR: Failed to send scan data to queue");
            }
        }

        // DETECCIÓN DE CRUCE POR CERO
        // Si el ángulo inicial de este paquete es menor que el del paquete anterior,
        // significa que el Lidar ha cruzado físicamente la marca de los 0 grados.
        if (start_angle < last_start_angle)
        {
            if (xQueueSend(lidar_data_queue, &end_of_scan, portMAX_DELAY) != pdTRUE)
            {
                Serial.println("ERROR: Failed to send end of scan to queue");
            }
            current_frame_is_end_of_scan = true;
        }
        
        // Actualizamos el ángulo para compararlo en la siguiente iteración
        last_start_angle = start_angle;
    }
    break;
    }
    return current_frame_is_end_of_scan;
}