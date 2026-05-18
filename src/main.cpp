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

// Motor / Nav
#include <AccelStepper.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <builtin_interfaces/msg/time.h>

// IMU
#include <sensor_msgs/msg/imu.h>

// Modules
// #include "../include/nema_motors.hpp"
#include "../include/xiaomi_lidar.hpp"
#include "../config/uros_wifi_config.h"
#include "../include/bmi160_sensor.hpp"
#include "../include/jga_motors.hpp"
#include "../include/emotions.hpp"

// ─── ROS entities ─────────────────────────────────────────────────────────────
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t sync_timer;
rcl_init_options_t init_options;
rcl_node_t node_lidar;
rcl_node_t node_motor;

// Un único publisher /tf compartido entre jga_motors e imu_sensor
rcl_publisher_t shared_tf_pub;

// ─── Sync ─────────────────────────────────────────────────────────────────────
const int sync_timeout_ms = 2000;
int64_t ros_synced_time_ms = 0;
int64_t ros_synced_time_ns = 0;
long ms_before_sync = 0;
unsigned int missing_syncs = 0;

// ─── Objects ──────────────────────────────────────────────────────────────────
TaskHandle_t rosTaskHandle;
XiaomiLidar xiaomi_lidar;
Bmi160Sensor imu_sensor;
JgaMotors jga_motors;
RobotEmotions robot_emotions;
rcl_timer_t timer_emotions;

// ─── Prototypes ───────────────────────────────────────────────────────────────
void vTaskMicroROS(void *pvParameters);
void sync_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void destroy_entities();
void create_entities();

// ─── Macros ───────────────────────────────────────────────────────────────────
#define RCCHECK(fn)                                         \
    {                                                       \
        rcl_ret_t temp_rc = fn;                             \
        if (temp_rc != RCL_RET_OK)                          \
        {                                                   \
            Serial.printf("[RCCHECK FAIL] line %d rc=%d\n", \
                          __LINE__, (int)temp_rc);          \
            error_loop();                                   \
        }                                                   \
    }
#define RCSOFTCHECK(fn)         \
    {                           \
        rcl_ret_t temp_rc = fn; \
        (void)temp_rc;          \
    }

void error_loop()
{
    jga_motors.emergency_stop();
    Serial.println("[ERROR_LOOP] Halted.");
    while (1)
    {
        delay(100);
    }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========== MINIBOT BOOT ==========");

    jga_motors.init();
    Serial.println("[SETUP] JGA motors OK");

    imu_sensor.init();
    Serial.println("[SETUP] IMU OK");

    robot_emotions.init();
    Serial.println("[SETUP] Emotions OK");

    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
    Serial.println("[SETUP] WiFi transport set");

    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 0);

    create_entities();
    Serial.println("[SETUP] Entities OK");

    xTaskCreatePinnedToCore(
        [](void *p)
        { xiaomi_lidar.SerialReadTask(p); },
        "SerialReadTask", 6144, NULL, 1, &xiaomi_lidar.serialTaskHandle, 0);

    xTaskCreatePinnedToCore(
        [](void *p)
        { xiaomi_lidar.RosPublishTask(p); },
        "RosPublishTask", 6144, NULL, 2, &xiaomi_lidar.rosPubTaskHandle, 1);

    xTaskCreatePinnedToCore(
        vTaskMicroROS, "vTaskMicroROS", 6144, (void *)1, 3, &rosTaskHandle, 1);

    if (rosTaskHandle == NULL)
    {
        Serial.println("[SETUP] ERROR: micro-ROS task creation failed");
        error_loop();
    }

    Serial.println("[SETUP] All tasks running.");
    Serial.println("===================================\n");
}

void loop() {}

// ─── MICRO-ROS TASK ───────────────────────────────────────────────────────────
void vTaskMicroROS(void *pvParameters)
{
    configASSERT(((uint32_t)pvParameters) == 1);
    uint32_t spin_count = 0;

    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
        spin_count++;

        // Log de estado cada ~5 s (250 ciclos x 20 ms = 5000 ms)
        if (spin_count % 250 == 0)
        {
            bool ok = (rmw_uros_ping_agent(50, 1) == RMW_RET_OK);
            Serial.printf("[uROS] spins=%u agent=%s missing=%u\n",
                          spin_count, ok ? "OK" : "FAIL", missing_syncs);
        }

        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
        {
            missing_syncs++;
        }
        else
        {
            if (missing_syncs > 0)
            {
                Serial.println("[uROS] Agent reconnected.");
                missing_syncs = 0;
            }
        }

        if (missing_syncs > 10)
        {
            Serial.println("[uROS] Agent lost — stopping motors and recreating entities.");
            jga_motors.emergency_stop();
            destroy_entities();
            create_entities();
            missing_syncs = 0;
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// ─── SYNC TIMER ───────────────────────────────────────────────────────────────
void sync_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL)
        return;
    rmw_uros_sync_session(sync_timeout_ms);

    if (rmw_uros_epoch_synchronized())
    {
        ros_synced_time_ms = rmw_uros_epoch_millis();
        ros_synced_time_ns = rmw_uros_epoch_nanos();
        ms_before_sync = millis();

        imu_sensor.ros_synced_time_ms = ros_synced_time_ms;
        imu_sensor.ms_before_sync = ms_before_sync;
        jga_motors.ros_synced_time_ms = ros_synced_time_ms;
        jga_motors.ros_synced_time_ns = ros_synced_time_ns;
        jga_motors.ms_before_sync = ms_before_sync;
        xiaomi_lidar.ros_synced_time_ms = ros_synced_time_ms;
        xiaomi_lidar.ros_synced_time_ns = ros_synced_time_ns;
        xiaomi_lidar.ms_before_sync = ms_before_sync;

        Serial.println("[SYNC] OK");
    }
    else
    {
        Serial.println("[SYNC] WARNING: not synchronized");
    }
}

// ─── DESTROY ENTITIES ─────────────────────────────────────────────────────────
void destroy_entities()
{
    Serial.println("[DESTROY] Destroying entities...");
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&xiaomi_lidar.laser_publisher, &node_lidar);
    rcl_subscription_fini(&jga_motors.cmd_vel_sub, &node_motor);
    rcl_publisher_fini(&jga_motors.odom_pub, &node_motor);
    rcl_publisher_fini(&imu_sensor.imu_pub, &node_motor);
    rcl_publisher_fini(&shared_tf_pub, &node_motor);
    rcl_subscription_fini(&robot_emotions.emotion_sub, &node_motor);

    rcl_timer_fini(&jga_motors.timer_odom);
    rcl_timer_fini(&sync_timer);
    rcl_timer_fini(&imu_sensor.timer_imu);
    rcl_timer_fini(&timer_emotions);

    rcl_node_fini(&node_lidar);
    rcl_node_fini(&node_motor);
    rclc_support_fini(&support);
    rclc_executor_fini(&executor);
    Serial.println("[DESTROY] Done.");
}

// ─── CREATE ENTITIES ──────────────────────────────────────────────────────────
void create_entities()
{
    Serial.println("[CREATE] Initializing...");
    memset(&imu_sensor.imu_msg, 0, sizeof(sensor_msgs__msg__Imu));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node_lidar, "minibot_lidar", "", &support));
    RCCHECK(rclc_node_init_default(&node_motor, "minibot_jga", "", &support));
    Serial.println("[CREATE] Nodes OK");

    RCCHECK(rclc_publisher_init_best_effort(
        &xiaomi_lidar.laser_publisher, &node_lidar,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan"));

    RCCHECK(rclc_publisher_init_default(
        &jga_motors.odom_pub, &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));

    RCCHECK(rclc_publisher_init_default(
        &imu_sensor.imu_pub, &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw"));

    RCCHECK(rclc_publisher_init_default(
        &shared_tf_pub, &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "/tf"));

    // Ambos objetos reciben el mismo handle (copia del struct, cuyo .impl
    // es un puntero opaco que no cambia tras la creación en micro-ROS)
    jga_motors.tf_pub = shared_tf_pub;
    imu_sensor.tf_pub = shared_tf_pub;

    Serial.println("[CREATE] Publishers OK");

    RCCHECK(rclc_subscription_init_default(
        &jga_motors.cmd_vel_sub, &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
    Serial.println("[CREATE] Subscription /cmd_vel OK");

    RCCHECK(rclc_subscription_init_default(
        &robot_emotions.emotion_sub, &node_motor,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/robot_emotion"));
    Serial.println("[CREATE] Subscription /robot_emotion OK");

    RCCHECK(rclc_timer_init_default(
        &timer_emotions, &support, RCL_MS_TO_NS(50), // 50ms = 20Hz
        [](rcl_timer_t *t, int64_t lct)
        {
            if (t != NULL)
                robot_emotions.update_task();
        }));

    RCCHECK(rclc_timer_init_default(
        &jga_motors.timer_odom, &support, RCL_MS_TO_NS(30),
        [](rcl_timer_t *t, int64_t lct)
        {
            jga_motors.timer_odometry_callback(t, lct);
        }));

    RCCHECK(rclc_timer_init_default(
        &sync_timer, &support, RCL_MS_TO_NS(5000), sync_timer_callback));

    RCCHECK(rclc_timer_init_default(
        &imu_sensor.timer_imu, &support, RCL_MS_TO_NS(50),
        [](rcl_timer_t *t, int64_t lct)
        {
            imu_sensor.timer_imu_callback(t, lct);
        }));

    Serial.println("[CREATE] Timers OK");

    // 6 handles: 2 subscription + 4 timers
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &jga_motors.cmd_vel_sub, &jga_motors.cmd_vel_msg,
        [](const void *msgin)
        {
            Serial.println("[EXEC] /cmd_vel received!"); // confirma que llega
            jga_motors.cmd_vel_sub_callback(msgin);
        },
        ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
        &executor, &robot_emotions.emotion_sub, &robot_emotions.emotion_msg,
        [](const void *msgin) {
            robot_emotions.emotion_sub_callback(msgin);
        }, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &timer_emotions));

    RCCHECK(rclc_executor_add_timer(&executor, &jga_motors.timer_odom));
    RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &imu_sensor.timer_imu));

    Serial.println("[CREATE] Executor OK — system ready.");
}