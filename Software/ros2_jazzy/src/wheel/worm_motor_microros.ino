#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#define LED_PIN 2
// --- TB6612FNG Pins for Motor A (Left) ---
#define AIN1 13
#define AIN2 14
#define PWMA 26

// --- TB6612FNG Pins for Motor B (Right) ---
#define BIN1 12
#define BIN2 27
#define PWMB 25

// --- Common Standby Pin ---
#define STBY 33

// --- micro-ROS variables ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Helper function to handle direction logic (Slightly upgraded to brake at 0)
void moveMotors(int speedA, int speedB) {
  // Control Motor A
  if (speedA > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else if (speedA < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); // Brake
  }
  analogWrite(PWMA, abs(speedA));

  // Control Motor B
  if (speedB > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else if (speedB < 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); // Brake
  }
  analogWrite(PWMB, abs(speedB));
}

// Callback: Runs every time Foxglove sends a joystick command
void twist_callback(const void * msgin) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Joystick values are typically -1.0 to 1.0
  float linear_x = twist_msg->linear.x;   // Forward/Back
  float angular_z = twist_msg->angular.z; // Left/Right turning

  // Differential drive mixing
  float left_ratio = linear_x - angular_z;
  float right_ratio = linear_x + angular_z;

  // Convert the -1.0 to 1.0 ratio into PWM speeds (-255 to 255)
  // constrain() prevents the math from going above 255 if you push forward AND turn at the same time
  int speedA = constrain(left_ratio * 255, -255, 255);
  int speedB = constrain(right_ratio * 255, -255, 255);

  // Send the calculated speeds to your custom function
  moveMotors(speedA, speedB);
}

void setup() {
  set_microros_transports(); // Uses USB cable to talk to Docker agent

  // Initialize all pins as outputs
  pinMode(LED_PIN, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable the motor driver
  digitalWrite(STBY, HIGH);

  // --- Initialize micro-ROS ---
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "dc_motor_node", "", &support);

  // Subscribe to the /cmd_vel topic
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // Set up the executor to listen for incoming messages
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &twist_callback, ON_NEW_DATA);
}

void loop() {
  // Check for new joystick messages every 10 milliseconds
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
