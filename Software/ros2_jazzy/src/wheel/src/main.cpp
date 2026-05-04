#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#define LED_PIN 2

// --- TB6612FNG Pins for FRONT Motors ---
#define FRONT_AIN1 32
#define FRONT_AIN2 14
#define FRONT_PWMA 13

#define FRONT_BIN1 25
#define FRONT_BIN2 26
#define FRONT_PWMB 27
#define FRONT_STBY 33

// --- TB6612FNG Pins for REAR Motors ---
#define REAR_AIN1 21
#define REAR_AIN2 22
#define REAR_PWMA 23

#define REAR_BIN1 18
#define REAR_BIN2 5
#define REAR_PWMB 17
#define REAR_STBY 19

// --- micro-ROS variables ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_command_time = 0;

// Helper function to control all 4 wheels simultaneously
void moveMotors(int speedLeft, int speedRight) {
  
  // --- LEFT SIDE (Front Motor A + Rear Motor A) ---
  if (speedLeft > 0) {
    digitalWrite(FRONT_AIN1, HIGH); digitalWrite(FRONT_AIN2, LOW);
    digitalWrite(REAR_AIN1,  HIGH); digitalWrite(REAR_AIN2,  LOW);
  } else if (speedLeft < 0) {
    digitalWrite(FRONT_AIN1, LOW);  digitalWrite(FRONT_AIN2, HIGH);
    digitalWrite(REAR_AIN1,  LOW);  digitalWrite(REAR_AIN2,  HIGH);
  } else {
    digitalWrite(FRONT_AIN1, LOW);  digitalWrite(FRONT_AIN2, LOW); // Brake
    digitalWrite(REAR_AIN1,  LOW);  digitalWrite(REAR_AIN2,  LOW); // Brake
  }
  analogWrite(FRONT_PWMA, abs(speedLeft));
  analogWrite(REAR_PWMA, abs(speedLeft));

  // --- RIGHT SIDE (Front Motor B + Rear Motor B) ---
  if (speedRight > 0) {
    digitalWrite(FRONT_BIN1, HIGH); digitalWrite(FRONT_BIN2, LOW);
    digitalWrite(REAR_BIN1,  HIGH); digitalWrite(REAR_BIN2,  LOW);
  } else if (speedRight < 0) {
    digitalWrite(FRONT_BIN1, LOW);  digitalWrite(FRONT_BIN2, HIGH);
    digitalWrite(REAR_BIN1,  LOW);  digitalWrite(REAR_BIN2,  HIGH);
  } else {
    digitalWrite(FRONT_BIN1, LOW);  digitalWrite(FRONT_BIN2, LOW); // Brake
    digitalWrite(REAR_BIN1,  LOW);  digitalWrite(REAR_BIN2,  LOW); // Brake
  }
  analogWrite(FRONT_PWMB, abs(speedRight));
  analogWrite(REAR_PWMB, abs(speedRight));
}

// Callback: Runs every time Foxglove sends a joystick command
void twist_callback(const void * msgin) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Flicker LED
  last_command_time = millis(); // Reset safety timer

  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = twist_msg->linear.x; 
  float angular_z = twist_msg->angular.z; 

  // Differential drive mixing
  float left_ratio = linear_x - angular_z;
  float right_ratio = linear_x + angular_z;

  // Convert -1.0/1.0 ratio to 0-255 PWM
  int speedLeft = constrain(left_ratio * 255, -255, 255);
  int speedRight = constrain(right_ratio * 255, -255, 255);

  moveMotors(speedLeft, speedRight);
}

void setup() {
  set_microros_transports(); 

  // Initialize all pins
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(FRONT_AIN1, OUTPUT); pinMode(FRONT_AIN2, OUTPUT); pinMode(FRONT_PWMA, OUTPUT);
  pinMode(FRONT_BIN1, OUTPUT); pinMode(FRONT_BIN2, OUTPUT); pinMode(FRONT_PWMB, OUTPUT);
  pinMode(FRONT_STBY, OUTPUT);
  
  pinMode(REAR_AIN1, OUTPUT); pinMode(REAR_AIN2, OUTPUT); pinMode(REAR_PWMA, OUTPUT);
  pinMode(REAR_BIN1, OUTPUT); pinMode(REAR_BIN2, OUTPUT); pinMode(REAR_PWMB, OUTPUT);
  pinMode(REAR_STBY, OUTPUT);

  // Enable both motor drivers
  digitalWrite(FRONT_STBY, HIGH);
  digitalWrite(REAR_STBY, HIGH);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_4x4_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &twist_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // DEADMAN SWITCH: Stop if no command for 500ms
  if (millis() - last_command_time > 500) {
    moveMotors(0, 0); 
  }
}