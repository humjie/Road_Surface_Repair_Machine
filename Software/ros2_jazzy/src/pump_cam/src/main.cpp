#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// ======================================================
// Error checking macros
// ======================================================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// ======================================================
// Pump pins/settings
// ======================================================
#define PUMP_PWM_PIN      33
#define PUMP_DIR_PIN      32

#define PUMP_PWM_FREQ     5000
#define PUMP_PWM_RES      8
#define PUMP_PWM_CHANNEL  0

// Inverted PWM logic for pump driver
#define PUMP_PWM_ON       0
#define PUMP_PWM_OFF      255

// ======================================================
// Cam motor pins/settings (TB6612FNG)
// ======================================================
#define CAM_IN1_PIN       5
#define CAM_IN2_PIN       18
#define CAM_PWM_PIN       19

#define CAM_IN3_PIN       23
#define CAM_IN4_PIN       22
#define CAM_PWM_PIN2      21

#define CAM_STBY_PIN      17

#define CAM_PWM_FREQ      5000
#define CAM_PWM_RES       8
#define CAM_PWM_CHANNEL1  1
#define CAM_PWM_CHANNEL2  2

#define CAM_MOTOR_SPEED   200
#define CAM_ROTATE_MS     500

// ======================================================
// Pump state
// ======================================================
bool pump_active = false;
unsigned long pump_stop_ms = 0;

// ======================================================
// Cam state
// ======================================================
bool cam_active = false;
unsigned long cam_stop_ms = 0;
bool cam_is_up = false;

// ======================================================
// micro-ROS entities
// ======================================================
rcl_subscription_t pump_cmd_sub;
rcl_subscription_t cam_cmd_sub;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

std_msgs__msg__Int32 msg_pump_cmd;
std_msgs__msg__String msg_cam_cmd;

// ======================================================
// Error loop
// ======================================================
void error_loop() {
  while (1) {
    delay(5000);
    ESP.restart();
  }
}

// ======================================================
// Pump Functions
// ======================================================
void pump_start(unsigned long duration_ms) {

  digitalWrite(PUMP_DIR_PIN, HIGH);

  // Start PWM
  ledcWrite(PUMP_PWM_CHANNEL, PUMP_PWM_ON);

  pump_active = true;
  pump_stop_ms = millis() + duration_ms;
}

void pump_stop() {

  digitalWrite(PUMP_DIR_PIN, LOW);

  // Stop PWM
  ledcWrite(PUMP_PWM_CHANNEL, PUMP_PWM_OFF);

  pump_active = false;
}

void update_pump() {

  if (pump_active && (long)(millis() - pump_stop_ms) >= 0) {
    pump_stop();
  }
}

// ======================================================
// Camera Motor Functions
// ======================================================
void cam_start(bool go_up) {

  // Prevent retrigger while already moving
  if (cam_active) {
    return;
  }

  // Direction control
  if (go_up) {

    digitalWrite(CAM_IN1_PIN, HIGH);
    digitalWrite(CAM_IN2_PIN, LOW);

    digitalWrite(CAM_IN3_PIN, HIGH);
    digitalWrite(CAM_IN4_PIN, LOW);

  } else {

    digitalWrite(CAM_IN1_PIN, LOW);
    digitalWrite(CAM_IN2_PIN, HIGH);

    digitalWrite(CAM_IN3_PIN, LOW);
    digitalWrite(CAM_IN4_PIN, HIGH);
  }

  // Apply PWM
  ledcWrite(CAM_PWM_CHANNEL1, CAM_MOTOR_SPEED);
  ledcWrite(CAM_PWM_CHANNEL2, CAM_MOTOR_SPEED);

  cam_active = true;
  cam_stop_ms = millis() + CAM_ROTATE_MS;

  cam_is_up = go_up;
}

void cam_stop() {

  // Stop PWM
  ledcWrite(CAM_PWM_CHANNEL1, 0);
  ledcWrite(CAM_PWM_CHANNEL2, 0);

  // Brake motors
  digitalWrite(CAM_IN1_PIN, LOW);
  digitalWrite(CAM_IN2_PIN, LOW);

  digitalWrite(CAM_IN3_PIN, LOW);
  digitalWrite(CAM_IN4_PIN, LOW);

  cam_active = false;
}

void update_cam() {

  if (cam_active && (long)(millis() - cam_stop_ms) >= 0) {
    cam_stop();
  }
}

// ======================================================
// ROS Callbacks
// ======================================================
void pump_cmd_callback(const void * msgin) {

  const std_msgs__msg__Int32 * msg =
    (const std_msgs__msg__Int32 *)msgin;

  int32_t duration_ms = msg->data;

  // Stop command
  if (duration_ms <= 0) {
    pump_stop();
    return;
  }

  // Start pump
  pump_start((unsigned long)duration_ms);
}

void cam_cmd_callback(const void * msgin) {

  const std_msgs__msg__String * msg =
    (const std_msgs__msg__String *)msgin;

  String cmd = String(msg->data.data);

  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "up") {

    if (!cam_active && !cam_is_up) {
      cam_start(true);
    }

  } else if (cmd == "down") {

    if (!cam_active && cam_is_up) {
      cam_start(false);
    }
  }
}

// ======================================================
// Setup
// ======================================================
void setup() {

  Serial.begin(115200);

  delay(2000);

  // ====================================================
  // Pump setup
  // ====================================================
  pinMode(PUMP_DIR_PIN, OUTPUT);

  digitalWrite(PUMP_DIR_PIN, LOW);

  ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RES);
  ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);

  ledcWrite(PUMP_PWM_CHANNEL, PUMP_PWM_OFF);

  // ====================================================
  // Cam motor setup
  // ====================================================
  pinMode(CAM_IN1_PIN, OUTPUT);
  pinMode(CAM_IN2_PIN, OUTPUT);

  pinMode(CAM_IN3_PIN, OUTPUT);
  pinMode(CAM_IN4_PIN, OUTPUT);

  pinMode(CAM_STBY_PIN, OUTPUT);

  digitalWrite(CAM_STBY_PIN, HIGH);

  // PWM setup
  ledcSetup(CAM_PWM_CHANNEL1, CAM_PWM_FREQ, CAM_PWM_RES);
  ledcAttachPin(CAM_PWM_PIN, CAM_PWM_CHANNEL1);

  ledcSetup(CAM_PWM_CHANNEL2, CAM_PWM_FREQ, CAM_PWM_RES);
  ledcAttachPin(CAM_PWM_PIN2, CAM_PWM_CHANNEL2);

  ledcWrite(CAM_PWM_CHANNEL1, 0);
  ledcWrite(CAM_PWM_CHANNEL2, 0);

  // ====================================================
  // micro-ROS setup
  // ====================================================
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // Initialize String message memory
  msg_cam_cmd.data.capacity = 20;
  msg_cam_cmd.data.size = 0;
  msg_cam_cmd.data.data =
    (char *) malloc(msg_cam_cmd.data.capacity);

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "pump_cam_node",
    "",
    &support));

  // ====================================================
  // Pump subscriber
  // ====================================================
  RCCHECK(rclc_subscription_init_default(
    &pump_cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "pump_cmd"));

  // ====================================================
  // Camera subscriber
  // ====================================================
  RCCHECK(rclc_subscription_init_default(
    &cam_cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "cam_control"));

  // ====================================================
  // Executor
  // ====================================================
  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    2,
    &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &pump_cmd_sub,
    &msg_pump_cmd,
    &pump_cmd_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &cam_cmd_sub,
    &msg_cam_cmd,
    &cam_cmd_callback,
    ON_NEW_DATA));
}

// ======================================================
// Main Loop
// ======================================================
void loop() {

  // Update timed actions
  update_pump();
  update_cam();

  // Handle ROS callbacks
  RCSOFTCHECK(
    rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(10)
    )
  );
}