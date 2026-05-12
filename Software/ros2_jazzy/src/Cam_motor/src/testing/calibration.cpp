// --- TB6612FNG Pin Definitions ---
#define AIN1 5   // Motor A Direction pin 1
#define AIN2 18  // Motor A Direction pin 2
#define PWMA 19  // Motor A Speed control (PWM)

#define BIN1 23  // Motor B Direction pin 1
#define BIN2 22  // Motor B Direction pin 2
#define PWMB 21  // Motor B Speed control (PWM)

#define STBY 17  // Standby pin

void setup() {
  Serial.begin(115200);

  // Initialize all motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  pinMode(STBY, OUTPUT);

  // Enable the motor driver (Crucial step: if LOW, neither motor will move)
  digitalWrite(STBY, HIGH);
  
  Serial.println("Motor driver enabled. Starting continuous rotation for both motors...");
}

void loop() {
  // --- Motor A Control ---
  // 1. Set the direction for Motor A (Forward)
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  // 2. Set the speed for Motor A (0 to 255)
  analogWrite(PWMA, 200);

  // --- Motor B Control ---
  // 1. Set the direction for Motor B (Forward)
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  // 2. Set the speed for Motor B (0 to 255)
  analogWrite(PWMB, 200);

  // Both motors will keep spinning indefinitely because we never 
  // write analogWrite to 0 or change the direction states.
  
  delay(100); // Small delay to keep the loop from running unnecessarily fast
}