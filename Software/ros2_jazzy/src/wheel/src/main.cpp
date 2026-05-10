#include <Arduino.h>

// --- TB6612FNG Pins for FRONT Motors ---
#define FRONT_AIN1 32
#define FRONT_AIN2 14
#define FRONT_PWMA 13

#define FRONT_BIN1 25
#define FRONT_BIN2 26
#define FRONT_PWMB 27
#define FRONT_STBY 33

// --- TB6612FNG Pins for REAR Motors ---
#define REAR_AIN1 21
#define REAR_AIN2 22
#define REAR_PWMA 23

#define REAR_BIN1 17
#define REAR_BIN2 16
#define REAR_PWMB 2
#define REAR_STBY 19

// Test Speed (0 to 255)
int testSpeed = 150; 

void stopAll() {
  analogWrite(FRONT_PWMA, 0);
  analogWrite(FRONT_PWMB, 0);
  analogWrite(REAR_PWMA, 0);
  analogWrite(REAR_PWMB, 0);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize all pins as OUTPUT
  pinMode(FRONT_AIN1, OUTPUT); pinMode(FRONT_AIN2, OUTPUT); pinMode(FRONT_PWMA, OUTPUT);
  pinMode(FRONT_BIN1, OUTPUT); pinMode(FRONT_BIN2, OUTPUT); pinMode(FRONT_PWMB, OUTPUT);
  pinMode(FRONT_STBY, OUTPUT);
  
  pinMode(REAR_AIN1, OUTPUT); pinMode(REAR_AIN2, OUTPUT); pinMode(REAR_PWMA, OUTPUT);
  pinMode(REAR_BIN1, OUTPUT); pinMode(REAR_BIN2, OUTPUT); pinMode(REAR_PWMB, OUTPUT);
  pinMode(REAR_STBY, OUTPUT);

  // Enable the drivers
  digitalWrite(FRONT_STBY, HIGH);
  digitalWrite(REAR_STBY, HIGH);
  
  Serial.println("Starting 4-Wheel Sequence Test...");
}

void loop() {
  // 1. FRONT LEFT (AIN)
  Serial.println("Testing: FRONT LEFT");
  digitalWrite(FRONT_AIN1, HIGH); digitalWrite(FRONT_AIN2, LOW);
  analogWrite(FRONT_PWMA, testSpeed);
  delay(2000);
  stopAll();
  delay(500);

  // 2. FRONT RIGHT (BIN)
  Serial.println("Testing: FRONT RIGHT");
  digitalWrite(FRONT_BIN1, HIGH); digitalWrite(FRONT_BIN2, LOW);
  analogWrite(FRONT_PWMB, testSpeed);
  delay(2000);
  stopAll();
  delay(500);

  // 3. REAR LEFT (AIN)
  Serial.println("Testing: REAR LEFT");
  digitalWrite(REAR_AIN1, HIGH); digitalWrite(REAR_AIN2, LOW);
  analogWrite(REAR_PWMA, testSpeed);
  delay(2000);
  stopAll();
  delay(500);

  // 4. REAR RIGHT (BIN)
  Serial.println("Testing: REAR RIGHT");
  digitalWrite(REAR_BIN1, HIGH); digitalWrite(REAR_BIN2, LOW);
  analogWrite(REAR_PWMB, testSpeed);
  delay(2000);
  stopAll();
  delay(1000);
}