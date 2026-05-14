#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define AIN1 5
#define AIN2 18
#define PWMA 19
#define BIN1 15
#define BIN2 22
#define PWMB 21
#define STBY 17

const int MOVE_DURATION    = 500;
const int MOTOR_SPEED      = 200;
bool      wheel_is_down    = false;
bool      execute_toggle   = false; // flag instead of blocking in callback

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

void moveMotors(int speedA, int speedB) {
  if      (speedA > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else if (speedA < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else                 { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  }
  analogWrite(PWMA, abs(speedA));

  if      (speedB > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else if (speedB < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else                 { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  }
  analogWrite(PWMB, abs(speedB));
}

void subscription_callback(const void * msgin) {
  execute_toggle = true; // just set flag, do NOT block here
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(STBY, HIGH);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "cam_motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "wheel_toggle"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  if (execute_toggle) {
    wheel_is_down = !wheel_is_down;

    if (wheel_is_down) {
      Serial.println("Wheel DOWN");
      moveMotors(MOTOR_SPEED, MOTOR_SPEED);
    } else {
      Serial.println("Wheel UP");
      moveMotors(-MOTOR_SPEED, -MOTOR_SPEED); // reverse direction
    }

    delay(MOVE_DURATION); // blocking is ok here since it's in loop(), not callback
    moveMotors(0, 0);
    execute_toggle = false;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}