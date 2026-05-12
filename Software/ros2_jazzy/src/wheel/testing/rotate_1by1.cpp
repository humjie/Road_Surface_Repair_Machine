#include <Arduino.h>

// --- TB6612FNG Pins for FRONT Motors ---
#define FRONT_AIN1 32
#define FRONT_AIN2 14
#define FRONT_PWMA 13

#define FRONT_BIN1 26
#define FRONT_BIN2 25
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

// --- Function Prototype for PlatformIO ---
void runMotor(int pin1, int pin2, int pwmPin, int speed);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Individual 4x4 Motor Diagnostic Test...");

  // Initialize FRONT Pins
  pinMode(FRONT_AIN1, OUTPUT); pinMode(FRONT_AIN2, OUTPUT); pinMode(FRONT_PWMA, OUTPUT);
  pinMode(FRONT_BIN1, OUTPUT); pinMode(FRONT_BIN2, OUTPUT); pinMode(FRONT_PWMB, OUTPUT);
  pinMode(FRONT_STBY, OUTPUT);

  // Initialize REAR Pins
  pinMode(REAR_AIN1, OUTPUT); pinMode(REAR_AIN2, OUTPUT); pinMode(REAR_PWMA, OUTPUT);
  pinMode(REAR_BIN1, OUTPUT); pinMode(REAR_BIN2, OUTPUT); pinMode(REAR_PWMB, OUTPUT);
  pinMode(REAR_STBY, OUTPUT);

  // Wake up both motor drivers
  digitalWrite(FRONT_STBY, HIGH);
  digitalWrite(REAR_STBY, HIGH);
}

void loop() {
  int testSpeed = 200; // Speed to test the motors at (0-255)
  int runTime = 2000;  // How long each motor spins (milliseconds)
  int pauseTime = 1000; // Pause between motors (milliseconds)

  // 1. Test FRONT LEFT Motor (Motor A on Front Driver)
  Serial.println("1. Spinning FRONT LEFT...");
  runMotor(FRONT_AIN1, FRONT_AIN2, FRONT_PWMA, testSpeed);
  delay(runTime);
  runMotor(FRONT_AIN1, FRONT_AIN2, FRONT_PWMA, 0); // Brake
  delay(pauseTime);

  // 2. Test FRONT RIGHT Motor (Motor B on Front Driver)
  Serial.println("2. Spinning FRONT RIGHT...");
  runMotor(FRONT_BIN1, FRONT_BIN2, FRONT_PWMB, testSpeed);
  delay(runTime);
  runMotor(FRONT_BIN1, FRONT_BIN2, FRONT_PWMB, 0); // Brake
  delay(pauseTime);

  // 3. Test REAR LEFT Motor (Motor A on Rear Driver)
  Serial.println("3. Spinning REAR LEFT...");
  runMotor(REAR_AIN1, REAR_AIN2, REAR_PWMA, testSpeed);
  delay(runTime);
  runMotor(REAR_AIN1, REAR_AIN2, REAR_PWMA, 0); // Brake
  delay(pauseTime);

  // 4. Test REAR RIGHT Motor (Motor B on Rear Driver)
  Serial.println("4. Spinning REAR RIGHT...");
  runMotor(REAR_BIN1, REAR_BIN2, REAR_PWMB, testSpeed);
  delay(runTime);
  runMotor(REAR_BIN1, REAR_BIN2, REAR_PWMB, 0); // Brake
  
  Serial.println("--- Test Sequence Complete. Restarting in 3 seconds ---");
  delay(3000);
}

// Universal helper function to drive ANY single motor
void runMotor(int pin1, int pin2, int pwmPin, int speed) {
  if (speed > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else if (speed < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW); // Brake
    digitalWrite(pin2, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}