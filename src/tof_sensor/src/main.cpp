#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ==========================================
//        MICRO-ROS INCLUDES & VARIABLES
// ==========================================
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point_stamped.h>

rcl_publisher_t publisher;
geometry_msgs__msg__PointStamped msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2 // Built-in LED on most ESP32 boards
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ==========================================
//        YOUR SCAN PARAMETERS
// ==========================================
VL53L1X sensor;

const int START_PIN = 4;           // Connect GPIO 4 to GND to start scan

const float X_SPEED_MM_S = 10.0;   // Speed of nozzle moving along X
const float Y_SPEED_MM_S = 10.0;   // Speed of nozzle stepping along Y
const float X_LENGTH_MM  = 100.0;  // Total width of the scan area
const float Y_LENGTH_MM  = 100.0;  // Total length of the scan area
const float SCAN_STEP_MM = 10.0;   // Distance between each scan (X and Y)
// ==========================================

// State Machine and Position Tracking
enum State { STANDBY, SCANNING_X, STEPPING_Y, DONE };
State currentState = STANDBY;

unsigned long lastTime = 0;
unsigned long startTime = 0;

float currentX = 0.0;
float currentY = 0.0;
int directionX = 1;     
float targetY = 0.0;    
int lastGridX = -1;     

void takeMeasurementAndPublish(unsigned long timestamp);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP); // Pin is HIGH by default, goes LOW when grounded

  Wire.begin(); // ESP32 defaults: SDA=21, SCL=22
  Wire.setClock(400000);

  // 1. Initialize ToF Sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    error_loop(); // Rapid blink if sensor fails
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); 
  sensor.startContinuous(50); 

  // 2. Initialize micro-ROS (Takes over the Serial port!)
  set_microros_serial_transports(Serial);
  delay(2000); 
  
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "tof_scanner_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
    "tof_scan_data"));

  // Initialize static message frame ID
  msg.header.frame_id.data = (char *)"scanner_frame";
  msg.header.frame_id.size = strlen(msg.header.frame_id.data);
  msg.header.frame_id.capacity = msg.header.frame_id.size + 1;
}

void loop() {
  unsigned long now = millis();
  
  // 1. Wait for Start Command (Hardware Pin Trigger)
  if (currentState == STANDBY) {
    if (digitalRead(START_PIN) == LOW) { // Detect wire touching GND
      startTime = millis();
      lastTime = startTime;
      currentState = SCANNING_X;
    }
    return; 
  }

  if (currentState == DONE) return;

  // 2. Calculate elapsed time (dt) for position integration
  float dt = (now - lastTime) / 1000.0; 
  lastTime = now;

  // 3. Update Position based on State
  if (currentState == SCANNING_X) {
    currentX += (X_SPEED_MM_S * dt * directionX);

    if ((directionX == 1 && currentX >= X_LENGTH_MM) || 
        (directionX == -1 && currentX <= 0.0)) {
      
      currentX = (directionX == 1) ? X_LENGTH_MM : 0.0;
      targetY = currentY + SCAN_STEP_MM;
      directionX *= -1; 
      currentState = STEPPING_Y;
    }
    
    int currentGridX = round(currentX / SCAN_STEP_MM);
    if (currentGridX != lastGridX) {
      takeMeasurementAndPublish(now - startTime);
      lastGridX = currentGridX;
    }

  } 
  else if (currentState == STEPPING_Y) {
    currentY += (Y_SPEED_MM_S * dt);

    if (currentY >= targetY) {
      currentY = targetY; 
      
      if (currentY > Y_LENGTH_MM) {
        currentState = DONE;
      } else {
        lastGridX = -1; 
        currentState = SCANNING_X;
      }
    }
  }
  
  sensor.read(); 
}

void takeMeasurementAndPublish(unsigned long timestamp) {
  uint16_t depth = sensor.ranging_data.range_mm;
  
  if (depth > 0 && depth < 4000) {
    msg.header.stamp.sec = timestamp / 1000;
    msg.header.stamp.nanosec = (timestamp % 1000) * 1000000;
    
    msg.point.x = currentX;
    msg.point.y = currentY;
    msg.point.z = (float)depth; 

    // Publish to the micro-ROS agent
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}