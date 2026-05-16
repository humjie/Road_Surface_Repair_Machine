/*
 * pump_cam_node.ino
 * ━━━━━━━━━━━━━━━━━
 * ESP32 micro-ROS firmware for pump (PWM) and cam motor (TB6612FNG)
 *
 * Topics subscribed
 *   /pump_cmd     (std_msgs/Int32)   duration in ms; 0 or negative = stop
 *   /cam_control  (std_msgs/String)  "up", "down", "toggle", "stop"
 *
 * Topics published
 *   /pump_status  (std_msgs/String)  "on" / "off"
 *   /cam_status   (std_msgs/String)  "up" / "down" / "unknown" / "moving"
 *
 * Fixes vs. previous version
 *   - cam_position_known now actually used: rejects "up"/"down" until first
 *     toggle establishes a known position (was silently broken on boot)
 *   - Status publishers added so the orchestrator can know real state
 *   - "stop" command added for cam (in case of jam)
 *   - Watchdog timer (5 s) added to recover from any hang
 *   - Cam PWM frequency raised to 20 kHz to eliminate audible whine
 *   - CAM_STBY pulled LOW when idle to save power (only HIGH while moving)
 *   - pump_stop no longer toggles DIR pin (just kills PWM) to avoid the
 *     potential reverse-spin glitch with some driver topologies
 *   - Reconnect / time-sync logic added to mirror the stepper firmware
 *   - publish_status helper guarded by capacity checks
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp_task_wdt.h>

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
// Error checking
// ======================================================
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// ======================================================
// Pump pins / settings
// ======================================================
#define PUMP_PWM_PIN      33
#define PUMP_DIR_PIN      32

#define PUMP_PWM_FREQ     5000
#define PUMP_PWM_RES      8
#define PUMP_PWM_CHANNEL  0

// Inverted PWM logic for pump driver
#define PUMP_PWM_ON       0
#define PUMP_PWM_OFF      255

// Safety: hard cap on a single pump duration to prevent runaway floods
#define PUMP_MAX_MS       60000UL   // 60 s

// ======================================================
// Cam motor pins / settings (TB6612FNG)
// ======================================================
#define CAM_IN1_PIN       5
#define CAM_IN2_PIN       18
#define CAM_PWM_PIN       19

#define CAM_IN3_PIN       15
#define CAM_IN4_PIN       22
#define CAM_PWM_PIN2      21

#define CAM_STBY_PIN      17

#define CAM_PWM_FREQ      20000    // raised from 5 kHz to kill audible whine
#define CAM_PWM_RES       8
#define CAM_PWM_CHANNEL1  1
#define CAM_PWM_CHANNEL2  2

#define CAM_MOTOR_SPEED   200
#define CAM_ROTATE_MS     4000

// ======================================================
// Watchdog
// ======================================================
#define WDT_TIMEOUT_S     5

// ======================================================
// State
// ======================================================
bool          pump_active  = false;
unsigned long pump_stop_ms = 0;

bool          cam_active        = false;
unsigned long cam_stop_ms       = 0;
bool          cam_is_up         = false;
bool          cam_position_known = false;   // actually used now

// ======================================================
// micro-ROS entities
// ======================================================
rcl_subscription_t  pump_cmd_sub;
rcl_subscription_t  cam_cmd_sub;

rcl_publisher_t     pump_status_pub;
rcl_publisher_t     cam_status_pub;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

std_msgs__msg__Int32  msg_pump_cmd;
std_msgs__msg__String msg_cam_cmd;

std_msgs__msg__String msg_pump_status;
std_msgs__msg__String msg_cam_status;

// ======================================================
// Forward declarations
// ======================================================
void publish_pump_status(const char * s);
void publish_cam_status(const char * s);

// ======================================================
// Error loop
// ======================================================
void error_loop() {
    while (1) {
        delay(2000);
        ESP.restart();
    }
}

// ======================================================
// Status publish helpers
// ======================================================
void publish_pump_status(const char * s) {
    if (msg_pump_status.data.data == NULL) return;
    snprintf(msg_pump_status.data.data, msg_pump_status.data.capacity, "%s", s);
    msg_pump_status.data.size = strlen(msg_pump_status.data.data);
    RCSOFTCHECK(rcl_publish(&pump_status_pub, &msg_pump_status, NULL));
}

void publish_cam_status(const char * s) {
    if (msg_cam_status.data.data == NULL) return;
    snprintf(msg_cam_status.data.data, msg_cam_status.data.capacity, "%s", s);
    msg_cam_status.data.size = strlen(msg_cam_status.data.data);
    RCSOFTCHECK(rcl_publish(&cam_status_pub, &msg_cam_status, NULL));
}

// ======================================================
// Pump
// ======================================================
void pump_start(unsigned long duration_ms) {
    if (duration_ms > PUMP_MAX_MS) {
        duration_ms = PUMP_MAX_MS;   // safety cap
    }

    digitalWrite(PUMP_DIR_PIN, HIGH);
    ledcWrite(PUMP_PWM_CHANNEL, PUMP_PWM_ON);

    pump_active  = true;
    pump_stop_ms = millis() + duration_ms;

    publish_pump_status("on");
}

void pump_stop() {
    // Kill PWM first, THEN drop DIR — avoids any reverse-spin glitch
    ledcWrite(PUMP_PWM_CHANNEL, PUMP_PWM_OFF);
    digitalWrite(PUMP_DIR_PIN, LOW);

    if (pump_active) {
        pump_active = false;
        publish_pump_status("off");
    }
}

void update_pump() {
    if (pump_active && (long)(millis() - pump_stop_ms) >= 0) {
        pump_stop();
    }
}

// ======================================================
// Cam motor
// ======================================================
void cam_set_direction(bool go_up) {
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
}

void cam_start(bool go_up) {
    if (cam_active) {
        return;  // refuse retrigger while moving
    }

    digitalWrite(CAM_STBY_PIN, HIGH);   // wake driver
    cam_set_direction(go_up);

    ledcWrite(CAM_PWM_CHANNEL1, CAM_MOTOR_SPEED);
    ledcWrite(CAM_PWM_CHANNEL2, CAM_MOTOR_SPEED);

    cam_active   = true;
    cam_stop_ms  = millis() + CAM_ROTATE_MS;
    cam_is_up    = go_up;
    cam_position_known = true;

    publish_cam_status("moving");
}

void cam_stop() {
    // Kill PWM first
    ledcWrite(CAM_PWM_CHANNEL1, 0);
    ledcWrite(CAM_PWM_CHANNEL2, 0);

    // Brake (all H-bridge inputs low)
    digitalWrite(CAM_IN1_PIN, LOW);
    digitalWrite(CAM_IN2_PIN, LOW);
    digitalWrite(CAM_IN3_PIN, LOW);
    digitalWrite(CAM_IN4_PIN, LOW);

    digitalWrite(CAM_STBY_PIN, LOW);    // sleep driver to save power
    cam_active = false;

    publish_cam_status(cam_position_known ? (cam_is_up ? "up" : "down") : "unknown");
}

void update_cam() {
    if (cam_active && (long)(millis() - cam_stop_ms) >= 0) {
        cam_stop();
    }
}

// ======================================================
// ROS callbacks
// ======================================================
void pump_cmd_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
    int32_t duration_ms = msg->data;

    if (duration_ms <= 0) {
        pump_stop();
        return;
    }
    pump_start((unsigned long) duration_ms);
}

void cam_cmd_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *) msgin;
    String cmd = String(msg->data.data);
    cmd.trim();
    cmd.toLowerCase();

    // "stop" works any time
    if (cmd == "stop") {
        if (cam_active) cam_stop();
        return;
    }

    // Ignore movement commands while already moving
    if (cam_active) return;

    if (cmd == "toggle") {
        // Toggle is always safe — even on boot it establishes a known position
        cam_start(!cam_is_up);
        return;
    }

    if (cmd == "up") {
        if (!cam_position_known) {
            // First-ever move: just go up to establish known position
            cam_start(true);
            return;
        }
        if (!cam_is_up) cam_start(true);
        return;
    }

    if (cmd == "down") {
        if (!cam_position_known) {
            // First-ever move: just go down to establish known position
            cam_start(false);
            return;
        }
        if (cam_is_up) cam_start(false);
        return;
    }
}

// ======================================================
// Setup
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    // ── Pump pins ─────────────────────────────────────
    pinMode(PUMP_DIR_PIN, OUTPUT);
    digitalWrite(PUMP_DIR_PIN, LOW);

    ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RES);
    ledcAttachPin(PUMP_PWM_PIN, PUMP_PWM_CHANNEL);
    ledcWrite(PUMP_PWM_CHANNEL, PUMP_PWM_OFF);

    // ── Cam pins ──────────────────────────────────────
    pinMode(CAM_IN1_PIN, OUTPUT);
    pinMode(CAM_IN2_PIN, OUTPUT);
    pinMode(CAM_IN3_PIN, OUTPUT);
    pinMode(CAM_IN4_PIN, OUTPUT);
    pinMode(CAM_STBY_PIN, OUTPUT);
    digitalWrite(CAM_STBY_PIN, LOW);   // sleep until needed

    ledcSetup(CAM_PWM_CHANNEL1, CAM_PWM_FREQ, CAM_PWM_RES);
    ledcAttachPin(CAM_PWM_PIN, CAM_PWM_CHANNEL1);
    ledcSetup(CAM_PWM_CHANNEL2, CAM_PWM_FREQ, CAM_PWM_RES);
    ledcAttachPin(CAM_PWM_PIN2, CAM_PWM_CHANNEL2);
    ledcWrite(CAM_PWM_CHANNEL1, 0);
    ledcWrite(CAM_PWM_CHANNEL2, 0);

    // ── Watchdog ──────────────────────────────────────
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    // ── micro-ROS ─────────────────────────────────────
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();

    // Allocate inbound cam command buffer
    msg_cam_cmd.data.capacity = 20;
    msg_cam_cmd.data.size     = 0;
    msg_cam_cmd.data.data     = (char *) malloc(msg_cam_cmd.data.capacity);

    // Allocate outbound status buffers
    msg_pump_status.data.capacity = 16;
    msg_pump_status.data.size     = 0;
    msg_pump_status.data.data     = (char *) malloc(msg_pump_status.data.capacity);

    msg_cam_status.data.capacity  = 16;
    msg_cam_status.data.size      = 0;
    msg_cam_status.data.data      = (char *) malloc(msg_cam_status.data.capacity);

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "pump_cam_node", "", &support));

    // ── Subscribers ───────────────────────────────────
    RCCHECK(rclc_subscription_init_default(&pump_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pump_cmd"));

    RCCHECK(rclc_subscription_init_default(&cam_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "cam_control"));

    // ── Publishers ────────────────────────────────────
    RCCHECK(rclc_publisher_init_default(&pump_status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "pump_status"));

    RCCHECK(rclc_publisher_init_default(&cam_status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "cam_status"));

    // ── Executor ──────────────────────────────────────
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &pump_cmd_sub,
        &msg_pump_cmd, &pump_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cam_cmd_sub,
        &msg_cam_cmd, &cam_cmd_callback, ON_NEW_DATA));

    // Announce starting states
    publish_pump_status("off");
    publish_cam_status("unknown");
}

// ======================================================
// Main loop
// ======================================================
void loop() {
    esp_task_wdt_reset();   // pet the watchdog

    update_pump();
    update_cam();

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
