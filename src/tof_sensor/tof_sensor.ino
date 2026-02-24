#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// ==========================================
//        YOUR SCAN PARAMETERS
// ==========================================
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
int directionX = 1;     // 1 for moving right (+), -1 for moving left (-)
float targetY = 0.0;    // Tracks the next Y row to step to
int lastGridX = -1;     // Ensures we only take one reading per grid point

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Error: Failed to initialize VL53L1X!");
    while (1); 
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); 
  sensor.startContinuous(50); 
  
  Serial.println("System Ready.");
  Serial.println("Array Format: [Timestamp_ms, X_Position, Y_Position, Depth_mm]");
  Serial.println("Send 'S' in the Serial Monitor at the EXACT moment your XY motors start.");
}

void loop() {
  unsigned long now = millis();
  
  // 1. Wait for Start Command
  if (currentState == STANDBY) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'S' || cmd == 's') {
        while(Serial.available() > 0) Serial.read(); // Clear buffer
        startTime = millis();
        lastTime = startTime;
        currentState = SCANNING_X;
        Serial.println("--- SCAN STARTED ---");
      }
    }
    return; // Don't run the rest of the loop until started
  }

  if (currentState == DONE) return; // Stop running once the area is covered

  // 2. Calculate elapsed time (dt) for position integration
  float dt = (now - lastTime) / 1000.0; // Convert to seconds
  lastTime = now;

  // 3. Update Position based on State
  if (currentState == SCANNING_X) {
    currentX += (X_SPEED_MM_S * dt * directionX);

    // Check if we hit the edge of the X boundary
    if ((directionX == 1 && currentX >= X_LENGTH_MM) || 
        (directionX == -1 && currentX <= 0.0)) {
      
      // Clamp to exact boundary to prevent drifting
      currentX = (directionX == 1) ? X_LENGTH_MM : 0.0;
      
      // Prepare to step down the Y axis
      targetY = currentY + SCAN_STEP_MM;
      directionX *= -1; // Flip X direction for the next pass
      currentState = STEPPING_Y;
    }
    
    // Check if we crossed a scanning threshold (e.g., every 10mm)
    int currentGridX = round(currentX / SCAN_STEP_MM);
    if (currentGridX != lastGridX) {
      takeMeasurementAndPrint(now - startTime);
      lastGridX = currentGridX;
    }

  } 
  else if (currentState == STEPPING_Y) {
    currentY += (Y_SPEED_MM_S * dt);

    // Check if we finished the Y step
    if (currentY >= targetY) {
      currentY = targetY; // Clamp to exact row
      
      // If we've reached the end of the board, stop.
      if (currentY > Y_LENGTH_MM) {
        currentState = DONE;
        Serial.println("--- SCAN COMPLETE ---");
      } else {
        // Reset X grid tracker and resume X scanning
        lastGridX = -1; 
        currentState = SCANNING_X;
      }
    }
  }
  
  // Keep sensor reading buffer clear even when not actively printing
  sensor.read(); 
}

void takeMeasurementAndPrint(unsigned long timestamp) {
  uint16_t depth = sensor.ranging_data.range_mm;
  
  // Only print valid depth readings
  if (depth > 0 && depth < 4000) {
    Serial.print(timestamp);
    Serial.print(", ");
    Serial.print(currentX, 1); // 1 decimal place
    Serial.print(", ");
    Serial.print(currentY, 1);
    Serial.print(", ");
    Serial.println(depth);
  }
}