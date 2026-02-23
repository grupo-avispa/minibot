#ifndef XIAOMI_LIDAR_HPP
#define XIAOMI_LIDAR_HPP

// C++
#include <vector>
#include <stdint.h>

// Arduino
#include <Arduino.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>

// Data structure representing a parsed frame from the Xiaomi Lidar
struct XiaomiLidarFrame {
    uint8_t frameHeader;          // Frame header (usually 0xAA)
    uint16_t frameLength;         // Total length of the frame
    uint8_t protocolVersion;      // Protocol version (typically 0x01)
    uint8_t frameType;            // Type of frame (e.g., data or health info)
    uint8_t commandWord;          // Command ID that defines the payload structure
    uint16_t parameterLength;     // Length of the parameters/payload
    std::vector<uint8_t> parameters; // Payload data (e.g., distances, angles)
    uint16_t checksum;            // Checksum for validating the frame
};

// Main class to interface with the Xiaomi Lidar
class XiaomiLidar {
public:
    // Constructor
    XiaomiLidar();

    // Destructor
    ~XiaomiLidar();

    // Starts the FreeRTOS task that reads data from the Lidar over Serial
    void SerialReadTask(void *pvParameters);

    // Handles status or connection-related checks (optional use)
    void HandleConnectionState();

    // Task that builds and publishes LaserScan messages using micro-ROS
    void RosPublishTask(void* pvParameters);

    // Fills a LaserScan message with distance data (angle/range conversions, etc.)
    void create_scan_msg(const std::vector<uint16_t>& distances);

    // Processes a complete Lidar frame (validates checksum, extracts data, pushes to queue)
    bool ProcessLidarFrame(const XiaomiLidarFrame& frame);

    // Public attributes (accessible from main or other components)
    rcl_publisher_t laser_publisher;               // ROS publisher for LaserScan
    sensor_msgs__msg__LaserScan scan_msg;          // Message used for publishing scan data

    // Time synchronization variables for micro-ROS
    int64_t ros_synced_time_ms = 0;                // Time in milliseconds from agent
    int64_t ros_synced_time_ns = 0;                // Time in nanoseconds from agent
    long ms_before_sync = 0;                       // Local millis() at time of last sync

    TaskHandle_t serialTaskHandle;                 // Task handle for serial reading
    TaskHandle_t rosPubTaskHandle;                 // Task handle for publishing to ROS

private:
    // Private helper methods (if needed)

    // FreeRTOS queue handle 
    QueueHandle_t lidar_data_queue;                // Queue used to pass scan data between tasks
    

    // Pin used for receiving serial data from the Lidar
    int lidar_rx_pin;
    // Pin to enable/disable the Lidar motor
    int motor_lidar_enable_pin; 

    // Constants defining frame format from the Lidar
    const uint8_t FRAME_HEADER;                    // Expected header byte (usually 0xAA)
    const uint8_t PROTOCOL_VERSION;                // Protocol version expected (usually 0x01)
    const uint8_t FRAME_TYPE;                      // Type of valid frame to process
    const uint16_t SCAN_STEPS;                     // Number of expected angular scan steps per frame
    const uint16_t SAMPLES_PER_FRAME;              // Expected number of distance samples in each frame

    // Scaling constants for converting raw data to usable units
    const float rotation_speed_scale;              // Converts rotation speed to RPM (0.05 rps * 60)
    const float angle_scale;                       // Converts angle LSB to degrees (0.01 deg/LSB)
    const float range_scale;                       // Converts range LSB to meters (0.25 mm per unit)

    // Frame tracking variables
    const uint16_t end_of_scan;                    // Marker to indicate the last sample of a scan
    uint16_t frame_index;                          // Counter to track scan frame sequence
};

#endif // XIAOMI_LIDAR_HPP
