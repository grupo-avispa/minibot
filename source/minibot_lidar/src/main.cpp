// C++
#include <vector>
#include <math.h>
#include <stdint.h>

// Arduino
#include <Arduino.h>

// uROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// Wifi Configuration
IPAddress agent_ip(192, 168, 0, 186);
size_t agent_port = 8888;

char ssid[] = "clarc1";
char psk[] = "robotclarc1";

// ROS entities
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t laser_publisher;
rcl_timer_t sync_timer;


// Laser scan message
sensor_msgs__msg__LaserScan scan_msg;// Example for 360 degrees

QueueHandle_t lidar_data_queue;
TaskHandle_t serialTaskHandle;
TaskHandle_t rosPubTaskHandle;
TaskHandle_t rosTaskHandle;

#define RXD2 17  // Rx pin for Serial2
#define TXD2 -1  // Tx pin for Serial2 (not used in this example)

// Data frame constants
const uint8_t FRAME_HEADER = 0xAA;
const uint8_t PROTOCOL_VERSION = 0x01;
const uint8_t FRAME_TYPE = 0x61;
const uint16_t SCAN_STEPS = 16;
const uint16_t SAMPLES_PER_FRAME = 17;

// Post processing constants
const float ROTATION_SPEED_SCALE = 0.05f * 60.0f; // Convert to RPM (LSB: 0.05 rps)
const float ANGLE_SCALE = 0.01f;                  // Convert to degrees (LSB: 0.01 degrees)
const float RANGE_SCALE = 0.25f * 0.001f;         // Convert to meters (LSB: 0.25 mm)

// Data variables
uint16_t initial_angle = 65535; // Initial angle in degrees
uint16_t initial_angle_lower_range, initial_angle_upper_range; // Initial angle lower and upper range in degrees
const uint16_t end_of_scan = 65535; // Used to indicate the end of a scan
uint16_t frame_index = 0;
// Data structures for scan samples
struct XiaomiLidarFrame {
    uint8_t frameHeader = 0;
    uint16_t frameLength = 0;
    uint8_t protocolVersion = 0;
    uint8_t frameType = 0;
    uint8_t commandWord = 0;
    uint16_t parameterLength = 0;
    std::vector<uint8_t> parameters;
    uint16_t checksum = 0;
};

// Synchronization variables (used for synchronizing ESP32 time with the PC agent)
const int sync_timeout_ms = 2000;
int64_t ros_synced_time_ms = 0;
int64_t ros_synced_time_ns = 0;
long ms_before_sync = 0;


// Function prototypes
void SerialReadTask(void *pvParameters);
void HandleConnectionState();
void RosPublishTask(void* pvParameters);
void create_scan_msg(const std::vector<uint16_t>& distances);
bool ProcessLidarFrame(const XiaomiLidarFrame& frame);
void vTaskMicroROS(void *pvParameters);
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);

// Macros for checking return of ROS2 functions and entering an infinite error loop in case of error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// Infinite error loop function. If something fails, the device will get stuck here
void error_loop() {
  while(1) {
    delay(100);
  }
}


/*
---------------------------------
|                               |  
|            SETUP              |
|                               |
---------------------------------
*/

void setup() {
    // Serial used to send data to PC
    Serial.begin(115200);       
    // Serial2 used to communicate with the Lidar
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  
    // Serial.println("Initialized ESP32 Xiaomi Lidar Scanner"); 

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    Serial.println("Micro-ROS WiFi transports set...");

    allocator = rcl_get_default_allocator();

    // create init_options and set domain ID
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 0);

    // create support structure
    RCCHECK(rclc_support_init_with_options(
      &support,
      0, NULL, &init_options, &allocator)); // 0 means no command line arguments

    // create node
    RCCHECK(rclc_node_init_default(
      &node,
      "minibot_lidar",
      "",
      &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
      &laser_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "/scan"));
    
    RCCHECK(rclc_timer_init_default(
      &sync_timer,
      &support,
      RCL_MS_TO_NS(5000), 
      sync_timer_callback));

    
    // create executor
    RCCHECK(rclc_executor_init(
      &executor, 
      &support.context, 
      1, // 1 means the executor can handle 1 entity (the timer)
      &allocator));
    
    RCCHECK(rclc_executor_add_timer(
      &executor,
      &sync_timer));

    // Create FreeRTOS queue for communication between tasks
    lidar_data_queue = xQueueCreate(600, sizeof(uint16_t));

    // Create FreeRTOS task for reading serial data from the Lidar
    xTaskCreatePinnedToCore(
      SerialReadTask, 
      "SerialReadTask",
      4096, 
      NULL, 
      1, 
      &serialTaskHandle, 
      0);
    Serial.println("Serial read task created...");
    
    // Create FreeRTOS task for crafting the scan msg and publishing 
    xTaskCreatePinnedToCore(
      RosPublishTask, 
      "RosPublishTask",
      4096, 
      NULL, 
      2, 
      &rosPubTaskHandle,
      1);
    Serial.println("ROS publish task created...");
    
    // Create FreeRTOS task for micro-ROS executor
    xTaskCreatePinnedToCore(
      vTaskMicroROS, 
      "vTaskMicroROS",
      42000, 
      (void *)1, 
      1, 
      &rosTaskHandle, 
      1);
    if (rosTaskHandle == NULL) {
      Serial.println("ERROR: Failed to create micro-ROS task");
      error_loop();
    }
    Serial.println("Micro-ROS task created...");

}

void loop() {
  // Main loop does nothing, tasks handle everything
}

/*
---------------------------------
|                               |  
|       RUNNING uROS TASK       |
|                               |
---------------------------------
*/

void vTaskMicroROS(void *pvParameters) {
  configASSERT(( (uint32_t) pvParameters ) == 1);
  // Initialize the micro-ROS executor
  for( ;; ){
    Serial.printf("Stack usage (executor task): %d words\n", uxTaskGetStackHighWaterMark(NULL));
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(500);
    // Serial.println("Micro-ROS executor spinning...");
  }
}

/*
---------------------------------
|            DATAFRAME          |  
|         SERIAL READING        |
|        FROM XIAOMI LIDAR      |
---------------------------------
*/

void SerialReadTask (void *pvParameters) {
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
  // Reserve space for parameters which enables fastest access. 
  // The number of parameters is variable, but we reserve a maximum size that will never be exceeded.
  frame.parameters.reserve(60);
  bool current_frame_is_end_of_scan = false; 
  while (true) {
        while (Serial2.available()) {
            uint8_t by = Serial2.read();
            switch (status) {
                case 0: // Frame Header
                    frame.frameHeader = by;
                    if (frame.frameHeader == FRAME_HEADER) {
                        checksum = by;
                        status = 1;
                        // Serial.println("Frame Header OK");
                    } else {
                        // Serial.println("ERROR: Frame Header Failed");
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
                    if (frame.protocolVersion == PROTOCOL_VERSION) {
                        status = 4;
                    } else {
                        Serial.println("ERROR: Protocol Version");
                        status = 0;
                    }
                    break;
                case 4: // Frame Type
                    frame.frameType = by;
                    checksum += by;
                    if (frame.frameType == FRAME_TYPE) {
                        status = 5;
                    } else {
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
                    // Serial.printf("Parameter Length: %d\n", frame.parameterLength);
                    checksum += by;
                    frame.parameters.clear();
                    status = 8;
                    break;
                case 8: // Parameters
                    frame.parameters.push_back(by);
                    checksum += by;
                    if (frame.parameters.size() == frame.parameterLength) {
                        status = 9;
                    }
                    break;
                case 9: // Checksum MSB
                    frame.checksum = by << 8;
                    status = 10;
                    break;
                case 10: // Checksum LSB
                    frame.checksum |= by;
                    if (frame.checksum == checksum) {
                        current_frame_is_end_of_scan = ProcessLidarFrame(frame);
                    } else {
                        Serial.println("ERROR: Checksum mismatch");
                    }
                    status = 0;
                    if (current_frame_is_end_of_scan) {
                        Serial.println("End of scan frame processed");
                        vTaskDelay(10);
                    }
                    break;
            }
            if (status < 10 && status > 0) {
                checksum %= 0xFFFF;
            }
        }
        // vTaskDelay(1);
    }
}

/*
---------------------------------
|            LASER SCAN         |  
|               uROS            |
|            PUBLISHER          |
---------------------------------
*/

void RosPublishTask(void* pvParameters) {
  std::vector<uint16_t> samples_vector;
  samples_vector.reserve(300); // Reserve space for 300 samples
  bool end_of_scan_received = false;
  uint16_t scan;
  // Initialize the scan message
  scan_msg.range_min = 0.05;
  scan_msg.range_max = 6.0;
  scan_msg.header.frame_id.data = "base_laser_link";
  scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);

  while (true) {
    if (xQueueReceive(lidar_data_queue, &scan, portMAX_DELAY) == pdPASS) {
      // Store the received scan data in the vector in case is not the end of scan
      if (scan == end_of_scan) {
        end_of_scan_received = true;
        Serial.println("[SCAN] End of scan received");
      }else{
        samples_vector.push_back(scan);
        // Serial.printf("[SCAN] Sample received: %d\n", scan);
      }
    }
    // Check if we received the end of scan signal
    if(end_of_scan_received) {
      end_of_scan_received = false;
      // Create the scan message with the collected distances
      create_scan_msg(samples_vector);
      samples_vector.clear(); // Clear the vector for the next scan
      // Publish the scan message
      rcl_publish(&laser_publisher, &scan_msg, NULL);
      Serial.println("[PUB] Laser scan message created and published");
    }
    // vTaskDelay(1);
  }
}


/*
---------------------------------
|            BUNCH OF           |  
|              UTIL             |
|            FUNCTIONS          |
---------------------------------
*/

bool ProcessLidarFrame(const XiaomiLidarFrame& frame) {
  bool current_frame_is_end_of_scan = false;
    switch (frame.commandWord) {
        case 0xAE: // Device Health Information
            {
                float rpm = frame.parameters[0] * ROTATION_SPEED_SCALE;
                Serial.printf("Device Health Information: \n");
                Serial.printf("RPM: %f\n", rpm);
            }
            break;
        case 0xAD: // Scan Data
            {
                uint16_t start_angle = (frame.parameters[3] << 8) | frame.parameters[4];
                // Serial.printf("Start Angle: %d\n", start_angle);

                frame_index++;

                int sample_cnt = static_cast<int>((frame.parameterLength - 5) / 3);
                for (int i = 0; i < sample_cnt; i++) {
                    uint16_t distance = (frame.parameters[5 + (i * 3) + 1] << 8) | frame.parameters[5 + (i * 3) + 2];
                    // Any distance can be end of scan
                    if (distance == end_of_scan) {
                        distance = end_of_scan - 1; 
                    }
                    if (xQueueSend(lidar_data_queue, &distance, portMAX_DELAY) != pdTRUE) {
                        Serial.println("ERROR: Failed to send scan data to queue");
                    }
                }
                // Serial.printf("SCAN: %d samples\n", sample_cnt);

                if (frame_index == (SCAN_STEPS)) {
                    // Send last angle before end of scan
                    if (start_angle == end_of_scan){
                      start_angle = end_of_scan - 1;
                    }
                    if (xQueueSend(lidar_data_queue, &start_angle, portMAX_DELAY) != pdTRUE) {
                        Serial.println("ERROR: Failed to send end angle scan to queue");
                    }
                    if (xQueueSend(lidar_data_queue, &end_of_scan, portMAX_DELAY) != pdTRUE) {
                        Serial.println("ERROR: Failed to send end of scan to queue");
                    }
                    // Serial.printf("SCAN: End of scan received\n");
                    frame_index = 0; // Reset frame index for the next scan
                    current_frame_is_end_of_scan = true;
                }
            }
            break;
    }
    return current_frame_is_end_of_scan;
}

void create_scan_msg(const std::vector<uint16_t>& samples) {
  // Update the scan message with the new data
  scan_msg.angle_max = samples.back() * ANGLE_SCALE * (M_PI / 180.0f) + (2 * M_PI) / SCAN_STEPS;
  scan_msg.angle_min = scan_msg.angle_max - (2 * M_PI);
  scan_msg.angle_increment = 2*M_PI / static_cast<float>(samples.size() - 1);
  scan_msg.header.stamp.sec = (ros_synced_time_ms + millis() - ms_before_sync) / 1000;
  scan_msg.header.stamp.nanosec = ros_synced_time_ns + (micros() * 1000) % 1000000000;

  // Deallocate the memory from heap
  if (scan_msg.ranges.data != nullptr) {
    // free(scan_msg.ranges.data);
    rosidl_runtime_c__float__Sequence__fini(&scan_msg.ranges);
  }
  // Allocate memory for the ranges data
  // scan_msg.ranges.data = (float*)malloc(samples.size() * sizeof(float));
  rosidl_runtime_c__float__Sequence__init(&scan_msg.ranges, samples.size() - 1);
  scan_msg.ranges.size = samples.size();
  for (size_t i = 0; i < samples.size() - 1; ++i) {
    float distance_meters = static_cast<float>(samples[i]) * static_cast<float>(RANGE_SCALE);
    scan_msg.ranges.data[i] = distance_meters;
  }
  Serial.printf("[SCAN] Scan message created with %d samples\n", samples.size());
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time){
    Serial.println("[TIMER] Sync timer callback called");
    if (timer == NULL)
    {
        Serial.println("Error in timer_callback: timer parameter is NULL\n");
        return;
    }
    // Synchronize time with the agent
    rmw_uros_sync_session(sync_timeout_ms);

    if (rmw_uros_epoch_synchronized())
    {
        // Get time in milliseconds or nanoseconds
        ros_synced_time_ms = rmw_uros_epoch_millis();
        ros_synced_time_ns = rmw_uros_epoch_nanos();
        ms_before_sync = millis();
        Serial.printf("Synchronized timestamp with PC agent\n");
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }
}
