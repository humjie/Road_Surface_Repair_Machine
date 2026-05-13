#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }

// Stepper driver pins for the Z axis.
#define Z_DIR 26
#define Z_STEP 27

static const int STEP_DELAY_US = 800;
static const int HOMING_DELAY_US = 1200;
static const float POSITION_SCALE_M = 0.001f;

rcl_publisher_t current_z_publisher;
rcl_publisher_t change_main_state_publisher;
rcl_subscription_t target_z_sub;
rcl_subscription_t main_state_sub;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

std_msgs__msg__Float32 msg_current_z;
std_msgs__msg__String msg_change_main_state;
std_msgs__msg__Float32 msg_target_z;
std_msgs__msg__String msg_main_state;

static float current_z_m = 0.0f;
static float target_z_m = 0.0f;
static bool homing_done = false;
static bool execute_home = false;
static bool execute_move = false;

void error_loop() {
  while (1) {
    delay(5000);
    ESP.restart();
  }
}

void publish_change_main_state(const char * state) {
  snprintf(msg_change_main_state.data.data, msg_change_main_state.data.capacity, "%s", state);
  msg_change_main_state.data.size = strlen(msg_change_main_state.data.data);
  RCSOFTCHECK(rcl_publish(&change_main_state_publisher, &msg_change_main_state, NULL));
}

void publish_current_z() {
  msg_current_z.data = current_z_m;
  RCSOFTCHECK(rcl_publish(&current_z_publisher, &msg_current_z, NULL));
}

void pulse_step(bool direction_high, int delay_us) {
  digitalWrite(Z_DIR, direction_high ? HIGH : LOW);
  digitalWrite(Z_STEP, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(Z_STEP, LOW);
  delayMicroseconds(delay_us);
}

void move_steps(long steps) {
  int direction = (steps >= 0) ? 1 : -1;
  long count = labs(steps);
  for (long i = 0; i < count; ++i) {
    pulse_step(direction > 0, STEP_DELAY_US);
  }
}

void home_z() {
  move_steps((long)lround(-current_z_m / POSITION_SCALE_M));
  current_z_m = 0.0f;
  homing_done = true;
}

void move_to_target() {
  long current_steps = (long)lround(current_z_m / POSITION_SCALE_M);
  long target_steps = (long)lround(target_z_m / POSITION_SCALE_M);
  long delta_steps = target_steps - current_steps;
  move_steps(delta_steps);
  current_z_m = target_z_m;
}

void target_z_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_z_m = msg->data;
  execute_move = true;
}

void main_state_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String state = String(msg->data.data);
  state.trim();
  if (state == "homing") {
    homing_done = false;
    execute_home = true;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    publish_current_z();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(Z_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  msg_current_z.data = 0.0f;
  msg_target_z.data = 0.0f;
  msg_change_main_state.data.capacity = 16;
  msg_change_main_state.data.data = (char *) malloc(msg_change_main_state.data.capacity * sizeof(char));
  msg_main_state.data.capacity = 16;
  msg_main_state.data.data = (char *) malloc(msg_main_state.data.capacity * sizeof(char));

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "z_axis_motor_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &current_z_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/current_z_pos"));

  RCCHECK(rclc_publisher_init_default(
    &change_main_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/change_main_state"));

  RCCHECK(rclc_subscription_init_default(
    &target_z_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/target_z"));

  RCCHECK(rclc_subscription_init_default(
    &main_state_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/main_state"));

  const unsigned int timer_timeout_ms = 100;
  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout_ms),
    timer_callback,
    true));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &target_z_sub, &msg_target_z, &target_z_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &main_state_sub, &msg_main_state, &main_state_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  rmw_uros_sync_session(1000);
}

void loop() {
  static unsigned long last_ping_time = 0;
  static unsigned long last_time_sync = 0;

  if (millis() - last_ping_time > 1000) {
    last_ping_time = millis();
    if (rmw_uros_ping_agent(100, 2) != RCL_RET_OK) {
      error_loop();
    }
  }

  if (millis() - last_time_sync > 60000) {
    rmw_uros_sync_session(100);
    last_time_sync = millis();
  }

  if (execute_home) {
    home_z();
    publish_change_main_state("free");
    execute_home = false;
  }

  if (execute_move && homing_done) {
    move_to_target();
    execute_move = false;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
