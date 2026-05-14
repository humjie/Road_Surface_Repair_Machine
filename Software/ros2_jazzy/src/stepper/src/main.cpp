#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/point.h>
#include <math.h>

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
#define X_DIR 12
#define X_STEP 14
#define X_MIN 18
#define X_MAX 19

// ===== Y AXIS =====
#define Y_DIR 17
#define Y_STEP 16
#define Y_MIN 32
#define Y_MAX 33

// ===== variables =====
const int STEP_DELAY    = 800;
const int HOMING_DELAY  = 1200;
const int RELEASE_DELAY = 1500;

// How often (in steps) to yield to the ROS executor during motion
// Tune this: smaller = more responsive comms, slightly more overhead
#define SPIN_EVERY_N_STEPS 200

// ===== positions =====
long xPos = 0, xMin = -750, xMax = 750;
long yPos = 0, yMin = -700, yMax = 700;

// Use preset min/max and skip endstop discovery when homing
const bool USE_PRESET_LIMITS = true;

bool homingDone = false;

// ========================================
// Endstop Checks
// ========================================
bool X_MIN_PRESSED() { return digitalRead(X_MIN) == LOW; }
bool X_MAX_PRESSED() { return digitalRead(X_MAX) == LOW; }
bool Y_MIN_PRESSED() { return digitalRead(Y_MIN) == LOW; }
bool Y_MAX_PRESSED() { return digitalRead(Y_MAX) == LOW; }

// ========================================
// Forward declarations
// ========================================
void ros_spin_some();

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
    if ((dir > 0 && X_MAX_PRESSED()) || (dir < 0 && X_MIN_PRESSED())) return;
    stepX(dir, delayUs);

    // FIX: Yield to ROS executor periodically so comms don't starve
    if (i % SPIN_EVERY_N_STEPS == 0) ros_spin_some();
  }
}

void moveYSteps(long steps, int delayUs) {
  int dir = (steps >= 0) ? 1 : -1;
  long count = abs(steps);

  for (long i = 0; i < count; i++) {
    if ((dir > 0 && Y_MAX_PRESSED()) || (dir < 0 && Y_MIN_PRESSED())) return;
    stepY(dir, delayUs);

    // FIX: Yield to ROS executor periodically
    if (i % SPIN_EVERY_N_STEPS == 0) ros_spin_some();
  }
}

// ========================================
// Homing X
// ========================================
bool homeX() {
  if (USE_PRESET_LIMITS) {
    // Move to center based on current position estimate.
    // NOTE: Only reliable if xPos is accurate (i.e. no missed steps / resets mid-move).
    moveXSteps(0 - xPos, STEP_DELAY);
    xPos = 0;
    return true;
  }

  if (X_MIN_PRESSED()) while (X_MIN_PRESSED()) stepX(1, RELEASE_DELAY);
  if (X_MAX_PRESSED()) while (X_MAX_PRESSED()) stepX(-1, RELEASE_DELAY);

  // Go to MAX
  while (!X_MAX_PRESSED()) { stepX(1, HOMING_DELAY); }

  while (X_MAX_PRESSED()) stepX(-1, RELEASE_DELAY);
  xPos = 0; // Define MAX side as reference zero

  // Go to MIN
  while (!X_MIN_PRESSED()) { stepX(-1, HOMING_DELAY); }
  long minPos = xPos;        // e.g. -1500
  long travel = abs(minPos); // total travel in steps

  while (X_MIN_PRESSED()) stepX(1, RELEASE_DELAY);

  // FIX: Center = move half the travel back from MIN toward MAX
  long stepsToCenter = travel / 2 - abs(xPos); // xPos is slightly > minPos after release
  moveXSteps(stepsToCenter, STEP_DELAY);

  xPos = 0;
  xMin = -(travel / 2);
  xMax =   travel / 2;

  return true;
}

// ========================================
// Homing Y
// ========================================
bool homeY() {
  if (USE_PRESET_LIMITS) {
    moveYSteps(0 - yPos, STEP_DELAY);
    yPos = 0;
    return true;
  }

  if (Y_MIN_PRESSED()) while (Y_MIN_PRESSED()) stepY(1, RELEASE_DELAY);
  if (Y_MAX_PRESSED()) while (Y_MAX_PRESSED()) stepY(-1, RELEASE_DELAY);

  // Go to MAX
  while (!Y_MAX_PRESSED()) { stepY(1, HOMING_DELAY); }

  while (Y_MAX_PRESSED()) stepY(-1, RELEASE_DELAY);
  yPos = 0;

  // Go to MIN
  while (!Y_MIN_PRESSED()) { stepY(-1, HOMING_DELAY); }
  long minPos = yPos;
  long travel = abs(minPos);

  while (Y_MIN_PRESSED()) stepY(1, RELEASE_DELAY);

  long stepsToCenter = travel / 2 - abs(yPos);
  moveYSteps(stepsToCenter, STEP_DELAY);

  yPos = 0;
  yMin = -(travel / 2);
  yMax =   travel / 2;

  return true;
}

// ========================================
// Move to Absolute XY with Endstop Checks
// (Bresenham line interpolation)
// ========================================
void moveToXY(long targetX, long targetY) {
  // Clamp targets to safe range
  targetX = constrain(targetX, xMin, xMax);
  targetY = constrain(targetY, yMin, yMax);

  long dx = targetX - xPos;
  long dy = targetY - yPos;
  int sx = (dx >= 0) ? 1 : -1;
  int sy = (dy >= 0) ? 1 : -1;
  dx = abs(dx);
  dy = abs(dy);

  if (dx == 0 && dy == 0) return;

  long stepCount = 0; // For periodic ROS spin

  if (dx >= dy) {
    long err = dx / 2;
    for (long i = 0; i < dx; i++) {
      if ((sx > 0 && X_MAX_PRESSED()) || (sx < 0 && X_MIN_PRESSED())) break;
      stepX(sx, STEP_DELAY);
      stepCount++;

      err -= dy;
      if (err < 0) {
        err += dx;
        if ((sy > 0 && Y_MAX_PRESSED()) || (sy < 0 && Y_MIN_PRESSED())) break;
        stepY(sy, STEP_DELAY);
        stepCount++;
      }

      // FIX: Yield to ROS executor periodically during motion
      if (stepCount % SPIN_EVERY_N_STEPS == 0) ros_spin_some();
    }
  } else {
    long err = dy / 2;
    for (long i = 0; i < dy; i++) {
      if ((sy > 0 && Y_MAX_PRESSED()) || (sy < 0 && Y_MIN_PRESSED())) break;
      stepY(sy, STEP_DELAY);
      stepCount++;

      err -= dx;
      if (err < 0) {
        err += dy;
        if ((sx > 0 && X_MAX_PRESSED()) || (sx < 0 && X_MIN_PRESSED())) break;
        stepX(sx, STEP_DELAY);
        stepCount++;
      }

      if (stepCount % SPIN_EVERY_N_STEPS == 0) ros_spin_some();
    }
  }
}

// ========================================
// ROS Entities & Variables
// ========================================
rcl_publisher_t    status_publisher;
rcl_publisher_t    pos_publisher;
rcl_publisher_t    current_xy_publisher;
rcl_publisher_t    change_main_state_publisher;
rcl_publisher_t    minmax_publisher;

rcl_subscription_t target_xy_sub;
rcl_subscription_t main_state_sub;

rcl_timer_t timer;

// Motion target (received from /target_xy)
geometry_msgs__msg__Point msg_target;

// Position publish message
geometry_msgs__msg__PointStamped msg_pos;

// Outgoing string messages
std_msgs__msg__String msg_status;
std_msgs__msg__String msg_minmax;
std_msgs__msg__String msg_change_main_state;

// FIX: Separate inbound buffer for /main_state subscriber
//      (previously shared with msg_change_main_state publisher — data corruption)
std_msgs__msg__String msg_main_state_in;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// State flags
long targetX_ros  = 0;
long targetY_ros  = 0;
bool execute_move = false;
bool execute_home = false;
char current_state[64] = "available";

// ========================================
// ROS Helper: spin executor briefly
// (called from inside step loops)
// ========================================
void ros_spin_some() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

// ========================================
// Error Loop
// ========================================
void error_loop() {
  while (1) {
    delay(5000);
    ESP.restart();
  }
}

// ========================================
// ROS Publish Helpers
// ========================================
void publish_status(const char* status) {
  int64_t time_ns = rmw_uros_epoch_nanos();
  int32_t sec = (int32_t)(time_ns / 1000000000);
  snprintf(msg_status.data.data, msg_status.data.capacity, "[%ld] %s", sec, status);
  msg_status.data.size = strlen(msg_status.data.data);
  RCSOFTCHECK(rcl_publish(&status_publisher, &msg_status, NULL));
}

void publish_change_main_state(const char* state) {
  snprintf(msg_change_main_state.data.data, msg_change_main_state.data.capacity, "%s", state);
  msg_change_main_state.data.size = strlen(msg_change_main_state.data.data);
  RCSOFTCHECK(rcl_publish(&change_main_state_publisher, &msg_change_main_state, NULL));
}

// ========================================
// Timer Callback (10 Hz)
// Publishes position, status, and minmax
// ========================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL) return;

  int64_t time_ns = rmw_uros_epoch_nanos();

  // --- Publish current XY position (PointStamped) ---
  msg_pos.header.stamp.sec     = (int32_t)(time_ns / 1000000000);
  msg_pos.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
  msg_pos.point.x = (double)xPos;
  msg_pos.point.y = (double)yPos;
  msg_pos.point.z = 0.0;
  RCSOFTCHECK(rcl_publish(&pos_publisher,        &msg_pos, NULL));
  RCSOFTCHECK(rcl_publish(&current_xy_publisher, &msg_pos, NULL));

  // --- Publish current state string ---
  int32_t sec = (int32_t)(time_ns / 1000000000);
  snprintf(msg_status.data.data, msg_status.data.capacity, "[%ld] %s", sec, current_state);
  msg_status.data.size = strlen(msg_status.data.data);
  RCSOFTCHECK(rcl_publish(&status_publisher, &msg_status, NULL));

  // --- Publish axis limits ---
  snprintf(
    msg_minmax.data.data,
    msg_minmax.data.capacity,
    "xmin:%ld xmax:%ld ymin:%ld ymax:%ld",
    xMin, xMax, yMin, yMax
  );
  msg_minmax.data.size = strlen(msg_minmax.data.data);
  RCSOFTCHECK(rcl_publish(&minmax_publisher, &msg_minmax, NULL));
}

// ========================================
// Subscription Callbacks
// ========================================
void target_xy_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  targetX_ros  = (long)msg->x;
  targetY_ros  = (long)msg->y;
  execute_move = true;
}

void main_state_callback(const void * msgin) {
  // FIX: Cast to the correct inbound message type (msg_main_state_in)
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String cmd = String(msg->data.data);
  cmd.trim();

  if (cmd == "homing") {
    homingDone   = false;
    execute_home = true;
    // FIX: Clear any pending move to avoid stale move firing right after homing
    execute_move = false;
    strncpy(current_state, "homing", sizeof(current_state) - 1);
    current_state[sizeof(current_state) - 1] = '\0';
    publish_status("homing");
  }
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(115200);

  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR,  OUTPUT);
  pinMode(X_MIN,  INPUT_PULLUP);
  pinMode(X_MAX,  INPUT_PULLUP);

  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR,  OUTPUT);
  pinMode(Y_MIN,  INPUT_PULLUP);
  pinMode(Y_MAX,  INPUT_PULLUP);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // --- Allocate string message buffers ---

  // Status publisher buffer
  msg_status.data.capacity = 150;
  msg_status.data.data      = (char *)malloc(msg_status.data.capacity * sizeof(char));
  msg_status.data.size      = 0;

  // Minmax publisher buffer
  msg_minmax.data.capacity = 80;
  msg_minmax.data.data      = (char *)malloc(msg_minmax.data.capacity * sizeof(char));
  msg_minmax.data.size      = 0;

  // FIX: Allocate the change_main_state publisher buffer (was missing — crash/UB)
  msg_change_main_state.data.capacity = 64;
  msg_change_main_state.data.data      = (char *)malloc(msg_change_main_state.data.capacity * sizeof(char));
  msg_change_main_state.data.size      = 0;

  // FIX: Allocate the dedicated inbound /main_state subscriber buffer
  msg_main_state_in.data.capacity = 64;
  msg_main_state_in.data.data      = (char *)malloc(msg_main_state_in.data.capacity * sizeof(char));
  msg_main_state_in.data.size      = 0;

  // Position message frame_id
  msg_pos.header.frame_id.capacity = 20;
  msg_pos.header.frame_id.data      = (char *)malloc(msg_pos.header.frame_id.capacity * sizeof(char));
  snprintf(msg_pos.header.frame_id.data, msg_pos.header.frame_id.capacity, "xy_stage");
  msg_pos.header.frame_id.size = strlen(msg_pos.header.frame_id.data);

  // --- ROS node & support ---
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "xy_stepper_node", "", &support));

  // --- Publishers ---
  RCCHECK(rclc_publisher_init_default(&status_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/xy_status"));

  RCCHECK(rclc_publisher_init_default(&pos_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped), "/current_pos"));

  RCCHECK(rclc_publisher_init_default(&current_xy_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped), "/current_xy_pos"));

  RCCHECK(rclc_publisher_init_default(&change_main_state_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/change_main_state"));

  RCCHECK(rclc_publisher_init_default(&minmax_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/xy_minmax"));

  // --- Subscribers ---
  RCCHECK(rclc_subscription_init_default(&target_xy_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "/target_xy"));

  RCCHECK(rclc_subscription_init_default(&main_state_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/main_state"));

  // --- Timer (10 Hz) ---
  RCCHECK(rclc_timer_init_default2(&timer, &support,
    RCL_MS_TO_NS(100), timer_callback, true));

  // --- Executor (2 subscriptions + 1 timer) ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &target_xy_sub,
    &msg_target, &target_xy_callback, ON_NEW_DATA));

  // FIX: Use the dedicated inbound buffer msg_main_state_in (not the publisher's buffer)
  RCCHECK(rclc_executor_add_subscription(&executor, &main_state_sub,
    &msg_main_state_in, &main_state_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Synchronize time with the ROS 2 agent
  rmw_uros_sync_session(1000);

  strncpy(current_state, "available", sizeof(current_state) - 1);
  current_state[sizeof(current_state) - 1] = '\0';
  publish_status("available");
}

// ========================================
// Loop
// ========================================
void loop() {
  static unsigned long last_ping_time  = 0;
  static unsigned long last_time_sync  = 0;

  // --- Ping agent to keep connection alive ---
  if (millis() - last_ping_time > 1000) {
    last_ping_time = millis(); // FIX: Update timestamp BEFORE ping so it stays fresh
    if (rmw_uros_ping_agent(100, 2) != RCL_RET_OK) {
      error_loop();
    }
  }

  // --- Resync clock every 60 s to prevent drift ---
  if (millis() - last_time_sync > 60000) {
    rmw_uros_sync_session(100);
    last_time_sync = millis();
  }

  // --- Handle HOME command ---
  if (execute_home) {
    homeX();
    homeY();
    homingDone   = true;
    execute_home = false;

    strncpy(current_state, "available", sizeof(current_state) - 1);
    current_state[sizeof(current_state) - 1] = '\0';
    publish_status("available");
    publish_change_main_state("free");
  }

  // --- Handle XY Move ---
  if (execute_move) {
    if (!homingDone) {
      // FIX: Discard stale move instead of silently queuing it forever
      publish_status("move rejected: homing not done");
      execute_move = false;
    } else {
      strncpy(current_state, "moving", sizeof(current_state) - 1);
      current_state[sizeof(current_state) - 1] = '\0';
      publish_status("moving");

      // FIX: last_ping_time refreshed before long motion so post-move ping doesn't fail
      last_ping_time = millis();

      moveToXY(targetX_ros, targetY_ros);

      strncpy(current_state, "available", sizeof(current_state) - 1);
      current_state[sizeof(current_state) - 1] = '\0';
      publish_status("available");
      execute_move = false;
    }
  }

  // --- Spin ROS executor ---
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}