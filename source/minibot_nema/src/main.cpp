#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <AccelStepper.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <tf2_msgs/msg/tf_message.h>
#include <builtin_interfaces/msg/time.h>

// ESP32 Pins and parameters for stepper motor control
#define STEP_PIN_L   26
#define DIR_PIN_L    25
#define STEP_PIN_R   27
#define DIR_PIN_R    14
#define MICRO_STEP   2   // Microstepping setting (hardware)
#define MAX_VEL      600 // Maximum velocity in steps/s

// WiFi credentials
char ssid[] = ""; // Replace with your WiFi SSID
char psk[]= ""; // Replace with your WiFi password

// MICRO ROS
IPAddress agent_ip(192, 168, 0, 186);
size_t agent_port = 8888;

// Publisher, Subscriber, Timer, and Node handles ...
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t tf_pub;
tf2_msgs__msg__TFMessage tf_msg;
rcl_timer_t timer_odom;
rcl_timer_t sync_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Synchronization variables (used for synchronizing ESP32 time with the PC agent)
const int sync_timeout_ms = 500;
int64_t synced_time_ms = 0;
int64_t synced_time_ns = 0;

// Hardware timer for motor control
hw_timer_t *timerMotor = NULL;

// Create stepper motor objects for left and right motors
AccelStepper stepper_l(AccelStepper::DRIVER, STEP_PIN_L, DIR_PIN_L);
AccelStepper stepper_r(AccelStepper::DRIVER, STEP_PIN_R, DIR_PIN_R);

// Robot constatns for odometry
const float wheel_radius = 0.05;      // wheel radius (5 cm)
const float wheel_base = 0.20;        // distance between wheels (20 cm)


// Robot state variables
float x = 0.0;
float y = 0.0;
float theta = 0.0;  // radians

// Previous steps for odometry calculation
long prev_steps_left = 0;
long prev_steps_right = 0;

// Conversion factors
const float steps_per_rev = 200.0 * MICRO_STEP; // Full turn in steps (200 steps for 1.8° stepper motor, multiplied by microstepping)
const float rad_per_step = 2.0 * PI / steps_per_rev;  // 1 step = 1 / (200*micro_step) of a full turn = 2pi / (200*micro_step) rad
const float meters_to_steps = 1.0 / (wheel_radius * rad_per_step); // Conversion factor from meters to steps
const float steps_to_meters = wheel_radius * rad_per_step; // Conversion factor from steps to meters

// Macros for checking return of ROS2 functions and entering an infinite error loop in case of error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Infinite error loop function. If something fails, the device will get stuck here
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Function prototypes
void subscription_callback(const void *msgin);
void timer_odometry_callback(rcl_timer_t * timer, int64_t last_call_time);
void vTaskMicroROS(void * pvParameters);
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void IRAM_ATTR timerMotorInterrupt();
void update_pose(const long *steps_left, const long *steps_right);

void setup() {
  Serial.begin(115200);

  // Configure motion
  stepper_l.setMaxSpeed(MAX_VEL);	
  stepper_l.setSpeed(0);
  stepper_l.setCurrentPosition(0);

  stepper_r.setMaxSpeed(MAX_VEL);	
  stepper_r.setSpeed(0);
  stepper_r.setCurrentPosition(0);
  
  Serial.println("Steppers initialized...");

  // MICROROS SETUP
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
    "minibot_nema",
    "",
    &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/minibot/odom"));
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &tf_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));
  
  // Allocate memory for TF array (only one transform in this case)
  tf_msg.transforms.capacity = 1;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped *)malloc(sizeof(geometry_msgs__msg__TransformStamped));
  if (tf_msg.transforms.data == NULL) {
      Serial.println("Failed to allocate memory for TF transform");
      error_loop();
  }
  memset(tf_msg.transforms.data, 0, sizeof(geometry_msgs__msg__TransformStamped));

  // create subscription
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/minibot/cmd_vel"));

  // create odometry timer
  RCCHECK(rclc_timer_init_default(
    &timer_odom,
    &support,
    RCL_MS_TO_NS(30),  // Timer for publishing odometry every 30 ms
    timer_odometry_callback));
  
  RCCHECK(rclc_timer_init_default(
    &sync_timer,
    &support,
    RCL_MS_TO_NS(1000), 
    sync_timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(
    &executor, 
    &support.context, 
    3, // 3 means the executor can handle 3 entities (2 timer and 1 subscription)
    &allocator)); 

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &cmd_vel_sub,
    &cmd_vel_msg,
    &subscription_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(
    &executor,
    &timer_odom));
  
  RCCHECK(rclc_executor_add_timer(
    &executor,
    &sync_timer));
  
  Serial.println("Micro-ROS node initialized...");

  // Create FreeRTOS task for micro-ROS executor
  xTaskCreatePinnedToCore(vTaskMicroROS, "vTaskMicroROS", 10000, (void *)1, 1, NULL, 0); // Run on core 0
  Serial.println("FreeRTOS task created...");

  // Create a hardware timer for the motor control
  timerMotor = timerBegin(0, 80, true); // Timer 0, clock divisor 80
  timerAttachInterrupt(timerMotor, &timerMotorInterrupt, true); // Attach the interrupt handling function
  timerAlarmWrite(timerMotor, 250, true); // Interrupt every 250 us 
  timerAlarmEnable(timerMotor); // Enable the alarm
  Serial.println("Hardware timer for motor control initialized...");
}


void loop() {}

void update_pose(const long *steps_left, const long *steps_right) {
  // Calculate delta steps since last update
  long delta_steps_left = *steps_left - prev_steps_left;
  long delta_steps_right = *steps_right - prev_steps_right;

  // Store previous steps for next iteration
  prev_steps_left = *steps_left;
  prev_steps_right = *steps_right;

  // Steps to meters conversion
  float dL = delta_steps_left * steps_to_meters; // Distance traveled by the left wheel
  float dR = delta_steps_right * steps_to_meters; // Distance traveled by the right wheel

  // robot odometry calculations
  float dC = (dR + dL) / 2.0;
  float dTheta = (dR - dL) / wheel_base;

  // Update robot position
  float delta_x = dC * cos(theta + dTheta / 2.0);
  float delta_y = dC * sin(theta + dTheta / 2.0);

  x += delta_x;
  y += delta_y;
  theta += dTheta;

  // Normalize theta to be within -PI to PI
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;
}

void IRAM_ATTR timerMotorInterrupt() {
  // This function is called every 500 us
  // It runs the stepper motor at the latest speed set by the cmd_vel subscription callback
  stepper_l.runSpeed();
  stepper_r.runSpeed();
}

//twist message cb
void subscription_callback(const void *msgin) {
  Serial.print("Received cmd_vel...\n");
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_velocity = msg->linear.x;  // m/s
  float angular_velocity = msg->angular.z; // rad/s

  // Calculate stepper speed in steps for differential drive wheel_base
  float left_step_speed = -((linear_velocity / wheel_radius) - 0.5 * wheel_base * (angular_velocity / wheel_radius)) / rad_per_step; 
  float right_step_speed = ((linear_velocity / wheel_radius) + 0.5 * wheel_base * (angular_velocity / wheel_radius)) / rad_per_step;

  stepper_l.setSpeed(left_step_speed);  
  stepper_r.setSpeed(right_step_speed); 
  Serial.print("Left speed: ");
  Serial.print(left_step_speed);
  Serial.print(" steps/s, Right speed: ");
  Serial.print(right_step_speed);
  Serial.println(" steps/s");
}

void timer_odometry_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    // Get current time
    odom_msg.header.stamp.sec = (synced_time_ms + millis()) / 1000;
    odom_msg.header.stamp.nanosec = synced_time_ns + (micros() * 1000) % 1000000000;
    // Compute current position based on stepper motor positions
    long stepper_l_pos = stepper_l.currentPosition();
    long stepper_r_pos = stepper_r.currentPosition();
    update_pose(&stepper_l_pos, &stepper_r_pos);
    // Fill odometry message
    odom_msg.header.frame_id.data = "odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.child_frame_id.data = "base_link";
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
    odom_msg.pose.pose.position.x = x;  // m
    odom_msg.pose.pose.position.y = y;  // m
    odom_msg.pose.pose.position.z = 0.0; // m
    odom_msg.pose.pose.orientation.x = 0.0; // Quaternion representation of orientation
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0); // z component of quaternion
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0); // w component of quaternion

    // Publish the odometry message
    rcl_publish(&odom_pub, &odom_msg, NULL);

    // Publish the TF message
    tf_msg.transforms.data[0].header.stamp.sec = odom_msg.header.stamp.sec;
    tf_msg.transforms.data[0].header.stamp.nanosec = odom_msg.header.stamp.nanosec;
    tf_msg.transforms.data[0].header.frame_id.data = "odom";
    tf_msg.transforms.data[0].header.frame_id.size = strlen(tf_msg.transforms.data[0].header.frame_id.data);
    tf_msg.transforms.data[0].child_frame_id.data = "base_link";
    tf_msg.transforms.data[0].child_frame_id.size = strlen(tf_msg.transforms.data[0].child_frame_id.data);
    tf_msg.transforms.data[0].transform.translation.x = x;  // m
    tf_msg.transforms.data[0].transform.translation.y = y;  // m
    tf_msg.transforms.data[0].transform.translation.z = 0.0; // m
    tf_msg.transforms.data[0].transform.rotation.x = 0.0; // Quaternion representation of orientation
    tf_msg.transforms.data[0].transform.rotation.y = 0.0;
    tf_msg.transforms.data[0].transform.rotation.z = odom_msg.pose.pose.orientation.z; // z component of quaternion
    tf_msg.transforms.data[0].transform.rotation.w = odom_msg.pose.pose.orientation.w; // w component of quaternion

    rcl_publish(&tf_pub, &tf_msg, NULL);

    // Serial.print("Published odometry ...\n");
  }
}

void vTaskMicroROS( void * pvParameters ){
  configASSERT( ( ( uint32_t ) pvParameters ) == 1 );

  for( ;; ){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));  // Run callbacks
    // Serial.println("Micro-ROS executor spinning...");
  }

}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time){
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
        synced_time_ms = rmw_uros_epoch_millis();
        synced_time_ns = rmw_uros_epoch_nanos();
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }
}