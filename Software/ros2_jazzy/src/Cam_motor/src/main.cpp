#include <Arduino.h>
// TB6612FNG Pins for Motor A (Left)
#define AIN1 5
#define AIN2 18
#define PWMA 19

// TB6612FNG Pins for Motor B (Right)
#define BIN1 23
#define BIN2 22
#define PWMB 21

// Common Standby Pin
#define STBY 17

void setup() {
  Serial.begin(115200);

  // Initialize all pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  pinMode(STBY, OUTPUT);

  // Enable the motor driver
  digitalWrite(STBY, HIGH);
  Serial.println("Dual Motor Test Initialized...");
}

void loop() {
  // // 1. Both Motors Forward (Speed 200 out of 255)
  // Serial.println("Both Motors: Forward");
  // moveMotors(200, 200);
  // delay(2000);

  // // 2. Stop
  // Serial.println("Both Motors: Stop");
  // moveMotors(0, 0);
  // delay(1000);

  // // 3. Both Motors Backward
  // Serial.println("Both Motors: Backward");
  // moveMotors(-200, -200);
  // delay(2000);

  // // 4. Stop
  // Serial.println("Both Motors: Stop");
  // moveMotors(0, 0);
  // delay(1000);
}

// Helper function to handle direction and speed logic
void moveMotors(int speedA, int speedB) {
  // Control Motor A [cite: 154, 155]
  if (speedA > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (speedA < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW); // Active brake
  }
  analogWrite(PWMA, abs(speedA));

  // Control Motor B [cite: 156, 157]
  if (speedB > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (speedB < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW); // Active brake
  }
  analogWrite(PWMB, abs(speedB));
}