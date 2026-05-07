#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h> 
#include <std_msgs/msg/int32.h>  
#include <geometry_msgs/msg/point_stamped.h> // NEW: For timestamps and X/Y combined
#include <geometry_msgs/msg/point.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Error checking macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ========================================
// XY A4988 Controller (Homing + Move XY)
// ========================================

// ===== X AXIS =====
#define Y_DIR 12
#define Y_STEP 14
#define Y_MIN 18
#define Y_MAX 19

// ===== Y AXIS =====
#define X_DIR 17
#define X_STEP 16
#define X_MIN 32
#define X_MAX 33

// ===== variables =====
const int STEP_DELAY = 800;
const int HOMING_DELAY = 1200;
const int RELEASE_DELAY = 1500;
const long MAX_STEPS = 40000;

// ===== positions =====
long xPos = 0, xMin = 0, xMax = 0;
long yPos = 0, yMin = 0, yMax = 0;

bool homingDone = false;

// ========================================
// Endstop Checks
// ========================================
bool X_MIN_PRESSED() { return digitalRead(X_MIN) == LOW; }
bool X_MAX_PRESSED() { return digitalRead(X_MAX) == LOW; }
bool Y_MIN_PRESSED() { return digitalRead(Y_MIN) == LOW; }
bool Y_MAX_PRESSED() { return digitalRead(Y_MAX) == LOW; }

// ========================================
// STEP control
// ========================================
void stepX(int dir, int delayUs) {
  digitalWrite(X_DIR, (dir > 0));
  digitalWrite(X_STEP, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(X_STEP, LOW);
  delayMicroseconds(delayUs);
  xPos += dir;
}

void stepY(int dir, int delayUs) {
  digitalWrite(Y_DIR, (dir > 0));
  digitalWrite(Y_STEP, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(Y_STEP, LOW);
  delayMicroseconds(delayUs);
  yPos += dir;
}

// ========================================
// Move X/Y by Steps with Endstop Checks
// ========================================
void moveXSteps(long steps, int delayUs) {
  int dir = (steps >= 0) ? 1 : -1;
  long count = abs(steps);

  for (long i = 0; i < count; i++) {
    if ((dir > 0 && X_MAX_PRESSED()) || (dir < 0 && X_MIN_PRESSED())) {
      return;
    }
    stepX(dir, delayUs);
  }
}

void moveYSteps(long steps, int delayUs) {
  int dir = (steps >= 0) ? 1 : -1;
  long count = abs(steps);

  for (long i = 0; i < count; i++) {
    if ((dir > 0 && Y_MAX_PRESSED()) || (dir < 0 && Y_MIN_PRESSED())) {
      return;
    }
    stepY(dir, delayUs);
  }
}

// ========================================
// Homing X
// ========================================
bool homeX() {
  if (X_MIN_PRESSED()) while (X_MIN_PRESSED()) stepX(1, RELEASE_DELAY);
  if (X_MAX_PRESSED()) while (X_MAX_PRESSED()) stepX(-1, RELEASE_DELAY);

  // go to MAX
  while (!X_MAX_PRESSED()) stepX(1, HOMING_DELAY);
  long maxPos = xPos;

  while (X_MAX_PRESSED()) stepX(-1, RELEASE_DELAY);

  xPos = 0;

  // go to MIN
  while (!X_MIN_PRESSED()) stepX(-1, HOMING_DELAY);
  long minPos = xPos;
  long travel = abs(xPos);

  while (X_MIN_PRESSED()) stepX(1, RELEASE_DELAY);

  long center = minPos + travel / 2;

  moveXSteps(center - xPos, STEP_DELAY);

  xPos = 0;
  xMin = -travel / 2;
  xMax =  travel / 2;

  return true;
}

// ========================================
// Homing Y
// ========================================
bool homeY() {
  if (Y_MIN_PRESSED()) while (Y_MIN_PRESSED()) stepY(1, RELEASE_DELAY);
  if (Y_MAX_PRESSED()) while (Y_MAX_PRESSED()) stepY(-1, RELEASE_DELAY);

  // go to MAX
  while (!Y_MAX_PRESSED()) stepY(1, HOMING_DELAY);
  long maxPos = yPos;

  while (Y_MAX_PRESSED()) stepY(-1, RELEASE_DELAY);

  yPos = 0;

  // go to MIN
  while (!Y_MIN_PRESSED()) stepY(-1, HOMING_DELAY);
  long minPos = yPos;
  long travel = abs(yPos);

  while (Y_MIN_PRESSED()) stepY(1, RELEASE_DELAY);

  long center = minPos + travel / 2;

  moveYSteps(center - yPos, STEP_DELAY);

  yPos = 0;
  yMin = -travel / 2;
  yMax =  travel / 2;

  return true;
}

// ========================================
// Move to Absolute XY with Endstop Checks
// ========================================
void moveToXY(long targetX, long targetY) {
  // Use Bresenham-like algorithm to interleave X/Y steps so motion is coordinated
  if (!homingDone) {
    return;
  }

  if (targetX < xMin || targetX > xMax ||
      targetY < yMin || targetY > yMax) {
    return;
  }

  long dx = targetX - xPos;
  long dy = targetY - yPos;
  int sx = (dx >= 0) ? 1 : -1;
  int sy = (dy >= 0) ? 1 : -1;
  dx = abs(dx);
  dy = abs(dy);

  if (dx == 0 && dy == 0) return;

  // If X is the dominant axis, step X each iteration and step Y when error accumulates.
  if (dx >= dy) {
    long err = dx / 2;
    for (long i = 0; i < dx; i++) {
      // Check endstops before stepping X
      if ((sx > 0 && X_MAX_PRESSED()) || (sx < 0 && X_MIN_PRESSED())) break;
      stepX(sx, STEP_DELAY);

      err -= dy;
      if (err < 0) {
        err += dx;
        // Check endstops before stepping Y
        if ((sy > 0 && Y_MAX_PRESSED()) || (sy < 0 && Y_MIN_PRESSED())) break;
        stepY(sy, STEP_DELAY);
      }
    }
  } else {
    // Y is dominant
    long err = dy / 2;
    for (long i = 0; i < dy; i++) {
      if ((sy > 0 && Y_MAX_PRESSED()) || (sy < 0 && Y_MIN_PRESSED())) break;
      stepY(sy, STEP_DELAY);

      err -= dx;
      if (err < 0) {
        err += dy;
        if ((sx > 0 && X_MAX_PRESSED()) || (sx < 0 && X_MIN_PRESSED())) break;
        stepX(sx, STEP_DELAY);
      }
    }
  }
}

// ========================================
// ROS Entities & Variables
// ========================================
rcl_publisher_t status_publisher;
rcl_publisher_t pos_publisher;

rcl_subscription_t target_xy_sub;
rcl_subscription_t cmd_sub;

rcl_timer_t timer;

geometry_msgs__msg__Point msg_target;
geometry_msgs__msg__PointStamped msg_pos; // Replaces separate X and Y msgs
std_msgs__msg__String msg_cmd;
std_msgs__msg__String msg_status;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// State management to prevent blocking callbacks
long targetX_ros = 0;
long targetY_ros = 0;
bool execute_move = false;
bool execute_home = false;
// Current continuous state string (published regularly)
char current_state[64] = "available";

// ========================================
// ROS Callbacks
// ========================================

void error_loop() {
  while(1) {
    delay(5000);
    ESP.restart();
  }
}

// Helper to replace Serial prints with ROS topic publishing
void publish_status(const char* status) {
  // Get synchronized time from the agent
  int64_t time_ns = rmw_uros_epoch_nanos();
  int32_t sec = time_ns / 1000000000;
  
  // Prepend the timestamp (seconds) to the status string
  snprintf(msg_status.data.data, msg_status.data.capacity, "[%ld] %s", sec, status);
  msg_status.data.size = strlen(msg_status.data.data);
  RCSOFTCHECK(rcl_publish(&status_publisher, &msg_status, NULL));
}

// Timer Callback: Continuously publishes the current X and Y coordinates with Timestamp
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    // 1. Get the current synchronized epoch time
    int64_t time_ns = rmw_uros_epoch_nanos();
    
    // 2. Assign time to the Header Stamp
    msg_pos.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    msg_pos.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

    // 3. Assign the actual coordinates
    msg_pos.point.x = xPos;
    msg_pos.point.y = yPos;
    msg_pos.point.z = 0.0; // Z is unused

    // 4. Publish
    rcl_publish(&pos_publisher, &msg_pos, NULL);

    // 5. Publish the current status continuously
    int32_t sec = time_ns / 1000000000;
    snprintf(msg_status.data.data, msg_status.data.capacity, "[%ld] %s", sec, current_state);
    msg_status.data.size = strlen(msg_status.data.data);
    RCSOFTCHECK(rcl_publish(&status_publisher, &msg_status, NULL));
  }
}

void target_xy_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  targetX_ros = (long)msg->x;
  targetY_ros = (long)msg->y;
  execute_move = true;
}

void cmd_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);
  cmd.trim();
  
  if (cmd == "HOME") {
    execute_home = true;
    strncpy(current_state, "homing", sizeof(current_state)-1);
    current_state[sizeof(current_state)-1] = '\0';
    publish_status("homing");
  } 
  else if (cmd == "POSE") {
    char status_str[50];
    snprintf(status_str, sizeof(status_str), "X: %ld Y: %ld", xPos, yPos);
    publish_status(status_str);
  }
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(115200);

  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_MIN, INPUT_PULLUP);
  pinMode(X_MAX, INPUT_PULLUP);

  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_MIN, INPUT_PULLUP);
  pinMode(Y_MAX, INPUT_PULLUP);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  
  // Allocate memory for string buffers
  msg_cmd.data.capacity = 50;
  msg_cmd.data.data = (char *) malloc(msg_cmd.data.capacity * sizeof(char));
  
  msg_status.data.capacity = 150; // Increased size to fit timestamp
  msg_status.data.data = (char *) malloc(msg_status.data.capacity * sizeof(char));

  // Allocate memory for the PointStamped frame_id string
  msg_pos.header.frame_id.capacity = 20;
  msg_pos.header.frame_id.data = (char *) malloc(msg_pos.header.frame_id.capacity * sizeof(char));
  snprintf(msg_pos.header.frame_id.data, msg_pos.header.frame_id.capacity, "xy_stage");
  msg_pos.header.frame_id.size = strlen(msg_pos.header.frame_id.data);

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "xy_stepper_node", "", &support));

  // --- Publishers ---
  RCCHECK(rclc_publisher_init_default(&status_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "xy_status"));

  RCCHECK(rclc_publisher_init_default(&pos_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped), "current_pos"));

  // --- Subscribers ---
  RCCHECK(rclc_subscription_init_default(&target_xy_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "target_xy"));
    
  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "xy_cmd"));

  // --- Timer Setup (10 Hz / 100ms) ---
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // --- Executor --- (Capacity 4: 3 Subscriptions + 1 Timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &target_xy_sub, &msg_target, &target_xy_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &msg_cmd, &cmd_callback, ON_NEW_DATA));
  
  // Synchronize time with the ROS 2 agent initially
  rmw_uros_sync_session(1000);

  strncpy(current_state, "available", sizeof(current_state)-1);
  current_state[sizeof(current_state)-1] = '\0';
  publish_status("available");
}

// ========================================
// Loop
// ========================================
void loop() {
  static unsigned long last_ping_time = 0;
  static unsigned long last_time_sync = 0;
  
  // Ping agent to keep connection alive
  if (millis() - last_ping_time > 1000) {
    last_ping_time = millis();
    if (rmw_uros_ping_agent(100, 2) != RCL_RET_OK) {
      error_loop();
    }
  }

  // Resynchronize time every 60 seconds to prevent clock drift
  if (millis() - last_time_sync > 60000) {
    rmw_uros_sync_session(100);
    last_time_sync = millis();
  }

  // Handle HOME command requested from ROS
  if (execute_home) {
    homeX();
    homeY();
    homingDone = true;
    strncpy(current_state, "available", sizeof(current_state)-1);
    current_state[sizeof(current_state)-1] = '\0';
    publish_status("available");
    
    targetX_ros = 0;
    targetY_ros = 0;
    execute_home = false;
  }

  // Handle XY Move requested from ROS
  if (execute_move) {
    if (!homingDone) {
      strncpy(current_state, "no home", sizeof(current_state)-1);
      current_state[sizeof(current_state)-1] = '\0';
      publish_status("no home");
      execute_move = false;
    } else {
      strncpy(current_state, "moving", sizeof(current_state)-1);
      current_state[sizeof(current_state)-1] = '\0';
      publish_status("moving");
      moveToXY(targetX_ros, targetY_ros);
      strncpy(current_state, "available", sizeof(current_state)-1);
      current_state[sizeof(current_state)-1] = '\0';
      publish_status("available");
      execute_move = false;
    }
  }

  // Spin the ROS Executor (Handles subscriptions and runs the timer callback)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}