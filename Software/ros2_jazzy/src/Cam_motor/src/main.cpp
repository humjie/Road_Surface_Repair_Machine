#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// --- TB6612FNG Pin Definitions ---
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// --- TB6612FNG Pin Definitions ---
#define AIN1 5
#define AIN2 18
#define PWMA 19
#define BIN1 23
#define BIN2 22
#define PWMB 21
#define STBY 17

// --- Cam Configuration ---
const int MOVE_DURATION = 500; // Time in milliseconds to reach the next position
const int MOTOR_SPEED = 200;    // PWM speed (0-255)
bool wheel_is_down = false;

// micro-ROS objects
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while(1) {
    digitalWrite(2, !digitalRead(2)); 
    delay(100);
  }
}

// Your original motor logic modified for simple movement
// --- Cam Configuration ---
const int MOVE_DURATION = 500; // Time in milliseconds to reach the next position
const int MOTOR_SPEED = 200;    // PWM speed (0-255)
bool wheel_is_down = false;

// micro-ROS objects
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() {
  while(1) {
    digitalWrite(2, !digitalRead(2)); 
    delay(100);
  }
}

// Your original motor logic modified for simple movement
void moveMotors(int speedA, int speedB) {
  // Motor A logic
  // Motor A logic
  if (speedA > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else if (speedA < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, abs(speedA));

  // Motor B logic
  // Motor B logic
  if (speedB > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else if (speedB < 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, abs(speedB));
}

// Callback: Triggered by Foxglove button
void subscription_callback(const void * msgin) {
  // Toggle state
  wheel_is_down = !wheel_is_down;

  if (wheel_is_down) {
    Serial.println("Action: Putting wheel DOWN");
    moveMotors(MOTOR_SPEED, MOTOR_SPEED); // Adjust polarity if needed
    delay(MOVE_DURATION);
    moveMotors(0, 0);
  } else {
    Serial.println("Action: Putting wheel UP");
    // If your cam rotates in one direction, keep MOTOR_SPEED positive. 
    // If it needs to reverse, use -MOTOR_SPEED.
    moveMotors(MOTOR_SPEED, MOTOR_SPEED); 
    delay(MOVE_DURATION);
    moveMotors(0, 0);
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Motor Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(2, OUTPUT); // Status LED

  digitalWrite(STBY, HIGH); // Enable driver

  // Initialize micro-ROS Serial Transport
  set_microros_serial_transports(Serial);

  delay(2000);
  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "cam_motor_node", "", &support));

  // Subscriber to /wheel_toggle
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "wheel_toggle"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}