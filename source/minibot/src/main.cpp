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
#include <rcl/error_handling.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw_microros/rmw_microros.h>

// Motor
#include <AccelStepper.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <builtin_interfaces/msg/time.h>

// Utils
#include "../include/nema_motors.hpp"
#include "../include/xiaomi_lidar.hpp"

// Wifi Configuration
IPAddress agent_ip(192, 168, 0, 186);
size_t agent_port = 8888;

char ssid[] = "clarc1";
char psk[] = "robotclarc1";

// ROS entities General
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t sync_timer;
rcl_init_options_t init_options;

// ROS entities Lidar
rcl_node_t node_lidar;

// Ros entities for Motor
rcl_node_t node_motor;

// Synchronization variables (used for synchronizing ESP32 time with the PC agent)
const int sync_timeout_ms = 2000;
int64_t ros_synced_time_ms = 0;
int64_t ros_synced_time_ns = 0;
long ms_before_sync = 0;
unsigned int missing_syncs = 0;

// Hardware timer for motor control
hw_timer_t *timerMotor = NULL;

// Spinning ros task handle
TaskHandle_t rosTaskHandle;

NemaMotors nema_motors; // Create an instance of the NemaMotors class
XiaomiLidar xiaomi_lidar; // Create an instance of the XiaomiLidar class

// Function prototypes

// Micro-ROS task and synchronization
void vTaskMicroROS(void *pvParameters);
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void destroy_entities();
void create_entities();

// Motor control ISR
void IRAM_ATTR timerMotorInterrupt();

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

    nema_motors.init(); // Initialize the Nema motors
    
    Serial.println("Steppers initialized...");

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    
    Serial.println("Micro-ROS WiFi transports set...");

    allocator = rcl_get_default_allocator();

    // create init_options and set domain ID
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 0);


    // Initialize micro-ROS
    create_entities();

    // Create FreeRTOS task for reading serial data from the Lidar
    xTaskCreatePinnedToCore(
        [](void *pvParameters) {
            xiaomi_lidar.SerialReadTask(pvParameters);
        }, 
        "SerialReadTask",
        4096, 
        NULL, 
        1, 
        &xiaomi_lidar.serialTaskHandle, 
        0);
    Serial.println("Serial read task created...");
    
    // Create FreeRTOS task for crafting the scan msg and publishing 
    xTaskCreatePinnedToCore(
        [](void *pvParameters) {
            xiaomi_lidar.RosPublishTask(pvParameters);
        },
        "RosPublishTask",
        4096, 
        NULL, 
        2, 
        &xiaomi_lidar.rosPubTaskHandle,
        1);
    Serial.println("ROS publish task created...");
    
    // Create FreeRTOS task for micro-ROS executor
    xTaskCreatePinnedToCore(
        vTaskMicroROS, 
        "vTaskMicroROS",
        10000, 
        (void *)1, 
        1, 
        &rosTaskHandle, 
        1);
    if (rosTaskHandle == NULL) {
        Serial.println("ERROR: Failed to create micro-ROS task");
        error_loop();
    }
    Serial.println("Micro-ROS task created...");

    // Create a hardware timer for the motor control
    timerMotor = timerBegin(0, 80, true); // Timer 0, clock divisor 80
    timerAttachInterrupt(timerMotor, &timerMotorInterrupt, true); // Attach the interrupt handling function
    timerAlarmWrite(timerMotor, 500, true); // Interrupt every 500 us 
    timerAlarmEnable(timerMotor); // Enable the alarm
    Serial.println("Hardware timer for motor control initialized...");
}


void loop() {
    // Main loop does nothing, tasks handle everything
}

/*
---------------------------------
|                               |  
|       Running uRos task       |
|                               |
---------------------------------
*/

void vTaskMicroROS(void *pvParameters) {
    configASSERT(( (uint32_t) pvParameters ) == 1);
    // Initialize the micro-ROS executor
    for( ;; ){
        //Serial.printf("Stack usage (executor task): %d words\n", uxTaskGetStackHighWaterMark(NULL));
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        // Check if the agent is reachable
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {  // timeout 100ms, 1 try
            Serial.println("Agent not reachable ...");
            missing_syncs++;
        }
        if (missing_syncs > 10) {
            // If the agent is not reachable, try to reconnect
            destroy_entities();
            Serial.println("Recreating entities...");
            create_entities();
            missing_syncs = 0; // Reset the missing syncs counter
        }

        vTaskDelay(100);
        // Serial.println("Micro-ROS executor spinning...");
    }
}

/*
---------------------------------
|                               |  
|        Motor control          | 
|        HW timer ISR           |
---------------------------------
*/

void IRAM_ATTR timerMotorInterrupt() {
    // This function is called every 500 us
    // It runs the stepper motor at the latest speed set by the cmd_vel subscription callback
    nema_motors.run_motors();
}

/*
---------------------------------
|                               |  
|       Synch with agent        | 
|        timer callback         |
---------------------------------
*/

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
        nema_motors.ros_synced_time_ms = ros_synced_time_ms;
        nema_motors.ros_synced_time_ns = ros_synced_time_ns;
        nema_motors.ms_before_sync = ms_before_sync;
        xiaomi_lidar.ros_synced_time_ms = ros_synced_time_ms;
        xiaomi_lidar.ros_synced_time_ns = ros_synced_time_ns;
        xiaomi_lidar.ms_before_sync = ms_before_sync;
        Serial.printf("Synchronized timestamp with PC agent\n");
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }
}

/*
---------------------------------
|                               |  
|      Destroying etities       | 
|                               |
---------------------------------
*/

void destroy_entities(){
    Serial.println("Destroying entities...");
    // Destroy the entities created
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_publisher_fini(&xiaomi_lidar.laser_publisher, &node_lidar);
    rcl_subscription_fini(&nema_motors.cmd_vel_sub, &node_motor);
    rcl_publisher_fini(&nema_motors.odom_pub, &node_motor);
    rcl_publisher_fini(&nema_motors.tf_pub, &node_motor);
    rcl_timer_fini(&nema_motors.timer_odom);
    rcl_timer_fini(&sync_timer);
    rcl_node_fini(&node_lidar);
    rcl_node_fini(&node_motor);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);
}

/*
---------------------------------
|                               |  
|        Create etities         | 
|                               |
---------------------------------
*/

void create_entities(){
    // create support structure
    RCCHECK(rclc_support_init_with_options(
        &support,
        0, // 0 means no command line arguments
        NULL, 
        &init_options, 
        &allocator)); 
    

    // create lidar node
    RCCHECK(rclc_node_init_default(
        &node_lidar,
        "minibot_lidar",
        "",
        &support));

    // create motor node
    RCCHECK(rclc_node_init_default(
        &node_motor,
        "minibot_nema",
        "",
        &support));

    Serial.println("Micro-ROS nodes initialized...");

    // create lidar publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &xiaomi_lidar.laser_publisher,
        &node_lidar,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan"));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &nema_motors.odom_pub,
        &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/minibot/odom"));
    
    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &nema_motors.tf_pub,
        &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "/tf"));

    // create subscription to cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &nema_motors.cmd_vel_sub,
        &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // create odometry timer
    RCCHECK(rclc_timer_init_default(
        &nema_motors.timer_odom,
        &support,
        RCL_MS_TO_NS(30),  // Timer for publishing odometry every 30 ms
        [](rcl_timer_t* timer, int64_t last_call_time) {
            nema_motors.timer_odometry_callback(timer, last_call_time);
        }));

    // create timer subscription
    RCCHECK(rclc_timer_init_default(
        &sync_timer,
        &support,
        RCL_MS_TO_NS(5000), 
        sync_timer_callback));

    
    // create executor
    RCCHECK(rclc_executor_init(
        &executor, 
        &support.context, 
        3, // 3 means the executor can handle 3 entity (2 timer and 1 subscription)
        &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &nema_motors.cmd_vel_sub,
        &nema_motors.cmd_vel_msg,
        [](const void *msgin) {
            // Serial.println("[SUB] cmd_vel message received");
            nema_motors.cmd_vel_sub_callback(msgin);
        },
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(
        &executor,
        &nema_motors.timer_odom));
    
    RCCHECK(rclc_executor_add_timer(
        &executor,
        &sync_timer));
    
    Serial.println("Micro-ROS node initialized...");
}