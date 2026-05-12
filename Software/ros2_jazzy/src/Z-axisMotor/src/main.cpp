#include <Arduino.h>

// Define ESP32 GPIO pin connections
const int dirPin = 26;
const int stepPin = 27;

// Define motor steps per revolution (200 is standard for a 1.8 degree motor)
const int stepsPerRev = 200;

void setup() {
  // Declare pins as output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  // 1. Set the spinning direction (HIGH = Clockwise)
  digitalWrite(dirPin, HIGH);

  // Spin the motor 1 revolution slowly
  for(int x = 0; x < stepsPerRev; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // Slower speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  delay(1000); // Wait 1 second

  // 2. Change the spinning direction (LOW = Counter-Clockwise)
  digitalWrite(dirPin, LOW);

  // Spin the motor 1 revolution quickly
  for(int x = 0; x < stepsPerRev; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Faster speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  delay(1000); // Wait 1 second before repeating
}