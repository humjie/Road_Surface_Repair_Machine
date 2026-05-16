/*
 * stepper_oneshot_node
 * ━━━━━━━━━━━━━━━━━━━━
 * Minimal micro-ROS firmware for the XY gantry with software acceleration.
 * Optimized for synchronized Bresenham stepping and higher speeds.
 *
 * CHANGES:
 *   - STEP_DELAY_US   : 500  → 200  (2.5× faster cruise speed)
 *   - START_DELAY_US  : 500  → 800  (softer ramp start to avoid stall)
 *   - ACCEL_STEPS     : 180  → 300  (longer ramp to reach higher cruise speed)
 *   - HOME_DELAY_US   : 500  → 300  (faster homing)
 *   - HOME_MAX_STEPS  : 2200 → 3000 (wider travel search)
 *   - X/Y MIN/MAX STEPS: ±1200 → ±1800 (wider scan range)
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <limits.h>

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// XY pins
#define X_DIR  12
#define X_STEP 14
#define X_MIN  18   // unused (switch spoiled)
#define X_MAX  19

#define Y_DIR  17
#define Y_STEP 16
#define Y_MIN  32   // unused
#define Y_MAX  33

// ── Motion tuning ──────────────────────────────────────────────────────────────
// Cruise step delay: lower = faster. 200 µs ≈ 5 000 steps/s.
// Reduce further only if your motor/driver can keep up without skipping.
#define STEP_DELAY_US    200   // was 500

// Starting delay at the bottom of the acceleration ramp.
// Must be ≥ STEP_DELAY_US. Higher value = gentler launch = less stall risk.
#define START_DELAY_US   800   // was 500

// Number of steps over which speed ramps up/down.
#define ACCEL_STEPS      300   // was 180

// Homing cruise delay (can be a bit faster than before).
#define HOME_DELAY_US    300   // was 500

// Back-off after hitting the end-stop.
#define BACKOFF_STEPS     10

// Maximum steps the homing search will travel before giving up.
#define HOME_MAX_STEPS  3000   // was 2200

#define WDT_TIMEOUT_S      5

// ── Logical travel limits (step units) ────────────────────────────────────────
// Widened from ±1200 to ±1800 to match the expanded scan window.
#define X_MIN_STEPS  -1800    // was -1200
#define X_MAX_STEPS   1800    // was  1200
#define Y_MIN_STEPS  -1800    // was -1200
#define Y_MAX_STEPS   1800    // was  1200

// ──────────────────────────────────────────────────────────────────────────────

rcl_publisher_t current_xy_pub;
rcl_publisher_t move_done_pub;
rcl_subscription_t target_xy_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

geometry_msgs__msg__Point msg_target;
geometry_msgs__msg__PointStamped msg_pos;
std_msgs__msg__String msg_move_done;

long xPos = 0;
long yPos = 0;
long targetX = 0;
long targetY = 0;
bool executing_move = false;
bool homed = false;

void error_loop() {
    while (1) {
        delay(2000);
        ESP.restart();
    }
}

void publish_move_done(const char * result) {
    if (msg_move_done.data.data == NULL) {
        return;
    }
    snprintf(msg_move_done.data.data, msg_move_done.data.capacity, "%s", result);
    msg_move_done.data.size = strlen(msg_move_done.data.data);
    RCSOFTCHECK(rcl_publish(&move_done_pub, &msg_move_done, NULL));
}

void publish_current_xy() {
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg_pos.header.stamp.sec     = (int32_t)(time_ns / 1000000000);
    msg_pos.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
    msg_pos.point.x = (double)xPos;
    msg_pos.point.y = (double)yPos;
    msg_pos.point.z = 0.0;
    RCSOFTCHECK(rcl_publish(&current_xy_pub, &msg_pos, NULL));
}

// ─────────────────────────── Stepper primitives ───────────────────────────────

inline void step_once(int stepPin, int dirPin, int dir) {
    digitalWrite(dirPin, (dir > 0) ? HIGH : LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);   // Minimum pulse width for most drivers
    digitalWrite(stepPin, LOW);
}

void step_x(int dir) {
    step_once(X_STEP, X_DIR, dir);
    xPos += dir;
}

void step_y(int dir) {
    step_once(Y_STEP, Y_DIR, dir);
    yPos += dir;
}

// ─────────────────────────────── Homing ──────────────────────────────────────

bool home_axis_max(int stepPin, int dirPin, int maxPin) {
    long count = 0;

    // Fast approach to the end-stop
    digitalWrite(dirPin, HIGH);
    while (digitalRead(maxPin) == HIGH && count < HOME_MAX_STEPS) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(HOME_DELAY_US);
        count++;
        esp_task_wdt_reset();
    }
    if (count >= HOME_MAX_STEPS) {
        return false;
    }

    // Back off
    for (int i = 0; i < BACKOFF_STEPS; i++) {
        step_once(stepPin, dirPin, -1);
        delayMicroseconds(HOME_DELAY_US);
        esp_task_wdt_reset();
    }

    // Slow second approach for repeatability
    digitalWrite(dirPin, HIGH);
    count = 0;
    while (digitalRead(maxPin) == HIGH && count < HOME_MAX_STEPS) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(HOME_DELAY_US * 2);   // half speed for accuracy
        count++;
        esp_task_wdt_reset();
    }
    if (count >= HOME_MAX_STEPS) {
        return false;
    }

    // Final back-off
    for (int i = 0; i < BACKOFF_STEPS; i++) {
        step_once(stepPin, dirPin, -1);
        delayMicroseconds(HOME_DELAY_US);
        esp_task_wdt_reset();
    }

    return true;
}

void home_xy() {
    bool x_ok = home_axis_max(X_STEP, X_DIR, X_MAX);
    xPos = X_MAX_STEPS - BACKOFF_STEPS;

    bool y_ok = home_axis_max(Y_STEP, Y_DIR, Y_MAX);
    yPos = Y_MAX_STEPS - BACKOFF_STEPS;

    publish_current_xy();

    if (x_ok && y_ok) {
        homed = true;
        publish_move_done("home");
    } else {
        homed = false;
        publish_move_done("home_fail");
    }
}

// ─────────────────── Movement: Bresenham + Linear Accel ──────────────────────

void move_to_xy(long newX, long newY) {
    long dx = newX - xPos;
    long dy = newY - yPos;
    int  sx = (dx >= 0) ? 1 : -1;
    int  sy = (dy >= 0) ? 1 : -1;
    dx = labs(dx);
    dy = labs(dy);

    if (dx == 0 && dy == 0) {
        publish_move_done("ok");
        return;
    }

    long err         = dx - dy;
    long total_steps = (dx > dy) ? dx : dy;
    long current_step = 0;
    long ramp_length  = ACCEL_STEPS;

    if (total_steps < ramp_length * 2) {
        ramp_length = total_steps / 2;
    }

    while (xPos != newX || yPos != newY) {
        esp_task_wdt_reset();

        // Linear acceleration profile
        int current_delay = STEP_DELAY_US;
        if (current_step < ramp_length) {
            // Accelerating: interpolate from START_DELAY_US → STEP_DELAY_US
            current_delay = START_DELAY_US
                - (int)((long)(START_DELAY_US - STEP_DELAY_US) * current_step / ramp_length);
        } else if (current_step > total_steps - ramp_length) {
            // Decelerating: interpolate from STEP_DELAY_US → START_DELAY_US
            long decel_step = current_step - (total_steps - ramp_length);
            current_delay = STEP_DELAY_US
                + (int)((long)(START_DELAY_US - STEP_DELAY_US) * decel_step / ramp_length);
        }

        // Bresenham: advance whichever axis (or both) is due
        long e2 = 2 * err;
        if (e2 > -dy && xPos != newX) {
            err -= dy;
            step_x(sx);
        }
        if (e2 < dx && yPos != newY) {
            err += dx;
            step_y(sy);
        }

        delayMicroseconds(current_delay);
        current_step++;
    }

    publish_current_xy();
    publish_move_done("ok");
}

// ─────────────────────────── ROS callback ────────────────────────────────────

void target_xy_callback(const void * msgin) {
    const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *) msgin;
    if (executing_move) {
        return;
    }

    long desiredX = (long)msg->x;
    long desiredY = (long)msg->y;

    // Clamp to travel limits
    if (desiredX < X_MIN_STEPS) desiredX = X_MIN_STEPS;
    if (desiredX > X_MAX_STEPS) desiredX = X_MAX_STEPS;
    if (desiredY < Y_MIN_STEPS) desiredY = Y_MIN_STEPS;
    if (desiredY > Y_MAX_STEPS) desiredY = Y_MAX_STEPS;

    targetX = desiredX;
    targetY = desiredY;
    executing_move = true;
}

// ─────────────────────────── Arduino setup/loop ───────────────────────────────

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

    digitalWrite(X_STEP, LOW);
    digitalWrite(Y_STEP, LOW);

    set_microros_serial_transports(Serial);

    while (rmw_uros_ping_agent(1000, 1) != RCL_RET_OK) {
        delay(100);
    }

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    allocator = rcl_get_default_allocator();

    msg_move_done.data.capacity = 32;
    msg_move_done.data.size     = 0;
    msg_move_done.data.data     = (char *)malloc(msg_move_done.data.capacity);

    msg_pos.header.frame_id.capacity = 20;
    msg_pos.header.frame_id.size     = 0;
    msg_pos.header.frame_id.data     = (char *)malloc(msg_pos.header.frame_id.capacity);
    if (msg_pos.header.frame_id.data != NULL) {
        snprintf(msg_pos.header.frame_id.data,
                 msg_pos.header.frame_id.capacity, "xy_stage");
        msg_pos.header.frame_id.size = strlen(msg_pos.header.frame_id.data);
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "stepper_oneshot_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &current_xy_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
        "/current_xy_pos"));
    RCCHECK(rclc_publisher_init_default(
        &move_done_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/move_done"));

    RCCHECK(rclc_subscription_init_default(
        &target_xy_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
        "/target_xy"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &target_xy_sub, &msg_target, &target_xy_callback, ON_NEW_DATA));

    home_xy();
}

void loop() {
    esp_task_wdt_reset();

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    if (executing_move) {
        move_to_xy(targetX, targetY);
        executing_move = false;
    }
}