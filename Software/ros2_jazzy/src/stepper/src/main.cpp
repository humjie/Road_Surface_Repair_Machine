/*
 Simple XY Stepper Test - NO ROS
 Just moves X and Y back and forth to verify hardware works.


 Uses your existing pin configuration:
   X: STEP=14, DIR=12
   Y: STEP=16, DIR=17
*/
#include <Arduino.h>

// ===== X AXIS =====
#define X_DIR  12
#define X_STEP 14


// ===== Y AXIS =====
#define Y_DIR  17
#define Y_STEP 16


// ===== Speed =====
const int STEP_DELAY = 2000;   // microseconds (lower = faster)
const int STEPS = 400;        // how many steps per move


void stepX(int dir) {
 digitalWrite(X_DIR, (dir > 0) ? HIGH : LOW);
 digitalWrite(X_STEP, HIGH);
 delayMicroseconds(STEP_DELAY);
 digitalWrite(X_STEP, LOW);
 delayMicroseconds(STEP_DELAY);
}


void stepY(int dir) {
 digitalWrite(Y_DIR, (dir > 0) ? HIGH : LOW);
 digitalWrite(Y_STEP, HIGH);
 delayMicroseconds(STEP_DELAY);
 digitalWrite(Y_STEP, LOW);
 delayMicroseconds(STEP_DELAY);
}


void setup() {
 Serial.begin(115200);
 delay(1000);
 Serial.println("=== Simple Stepper Test ===");


 pinMode(X_STEP, OUTPUT);
 pinMode(X_DIR,  OUTPUT);
 pinMode(Y_STEP, OUTPUT);
 pinMode(Y_DIR,  OUTPUT);


 Serial.println("Pins configured. Starting movement in 2 seconds...");
 delay(2000);
}


void loop() {
 // --- X forward ---
 Serial.println("X forward");
 for (int i = 0; i < STEPS; i++) stepX(1);
 delay(500);


 // --- X backward ---
 Serial.println("X backward");
 for (int i = 0; i < STEPS; i++) stepX(-1);
 delay(500);


 // --- Y forward ---
 Serial.println("Y forward");
 for (int i = 0; i < STEPS; i++) stepY(1);
 delay(500);


 // --- Y backward ---
 Serial.println("Y backward");
 for (int i = 0; i < STEPS; i++) stepY(-1);
 delay(1000);


 Serial.println("--- cycle done ---\n");
}