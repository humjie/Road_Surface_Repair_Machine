#include <Arduino.h>
#include <Stepper.h>

// Change this to match your specific motor (200 is standard for NEMA 17)
const int stepsPerRev = 200; 

// TB6612FNG Pins connected to ESP32
#define AIN1 21
#define AIN2 22
#define BIN1 18
#define BIN2 5
#define PWMA 23
#define PWMB 17
#define STBY 19

// Initialize the Stepper library. 
// The pin sequence for TB6612FNG is typically AIN1, BIN1, AIN2, BIN2
Stepper myStepper(stepsPerRev, AIN1, BIN1, AIN2, BIN2);

void setup() {
  Serial.begin(115200);

  // Set up the power and standby pins
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable the motor driver and send full power to both channels
  digitalWrite(PWMA, HIGH);
  digitalWrite(PWMB, HIGH);
  digitalWrite(STBY, HIGH);

  // Set the motor speed (in RPM)
  myStepper.setSpeed(60); 
  
  Serial.println("Stepper Motor Test Initialized.");
}

void loop() {
  Serial.println("Spinning 1 full revolution forward...");
  // myStepper.step(stepsPerRev); // Move 200 steps forward
  // delay(1000);

  // Serial.println("Spinning half revolution backward...");
  // myStepper.step(-stepsPerRev / 2); // Move 100 steps backward
  // delay(1000);
}