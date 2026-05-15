#include <Arduino.h>

#define Z_DIR  26
#define Z_STEP 27

static const int STEP_DELAY_US = 800;
static const float MM_PER_STEP = 0.1f; // adjust to your leadscrew/microstepping

void move_mm(float mm) {
  long steps = (long)roundf(mm / MM_PER_STEP);
  bool dir = steps >= 0;
  steps = labs(steps);

  digitalWrite(Z_DIR, dir ? HIGH : LOW);
  delayMicroseconds(10); // settle time for direction

  for (long i = 0; i < steps; i++) {
    digitalWrite(Z_STEP, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(Z_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }

  Serial.printf("Moved %.2f mm (%ld steps)\n", mm, steps);
}

void setup() {
  Serial.begin(115200);
  pinMode(Z_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  digitalWrite(Z_DIR, LOW);
  digitalWrite(Z_STEP, LOW);
  Serial.println("Z-axis test ready.");
  Serial.println("Commands: +10  -5  +0.5  etc. (mm)");
}

void loop() {
  move_mm(10); // example move, replace with serial command parsing as needed
  delay(2000);
  move_mm(-10);
  delay(2000);
  // if (Serial.available()) {
  //   String input = Serial.readStringUntil('\n');
  //   input.trim();
  //   if (input.length() > 0) {
  //     float mm = input.toFloat();
  //     if (mm != 0.0f) {
  //       Serial.printf("Moving %.2f mm...\n", mm);
  //       move_mm(mm);
  //       Serial.println("Done.");
  //     } else {
  //       Serial.println("Invalid input. Example: +10 or -5");
  //     }
  //   }
  // }
}