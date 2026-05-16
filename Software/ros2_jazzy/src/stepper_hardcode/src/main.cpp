/*
 * xy_scan_then_direction.ino
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * Waits for the character 'S' over USB Serial (sent by the Pi ROS 2
 * node when Foxglove publishes True on /start_scan), then runs:
 *   Phase 1: Snake-pattern sweep across the work area
 *   Phase 2: Return to center of the sweep area
 *   Phase 3: Move in a hardcoded direction by a set distance
 *
 * After completion it waits for 'S' again — no reset needed.
 *
 *   ⚠️  Start the gantry at physical (0, 0) before powering on.
 *   ⚠️  No endstop safety checks — ensure all moves are within safe travel.
 *
 * Pins:
 *   X_DIR=12  X_STEP=14
 *   Y_DIR=17  Y_STEP=16
 */

#include <Arduino.h>

// ────────────────────────────────────────────────────────────────────
// PINS
// ────────────────────────────────────────────────────────────────────
#define X_DIR    12
#define X_STEP   14

#define Y_DIR    17
#define Y_STEP   16

// ────────────────────────────────────────────────────────────────────
// SNAKE-SWEEP DIMENSIONS  (steps — no endstops, make sure they fit!)
// ────────────────────────────────────────────────────────────────────
#define X_LENGTH_STEPS    1000
#define Y_LENGTH_STEPS    1000
#define STEP_SIZE_Y       100

// ────────────────────────────────────────────────────────────────────
// PHASE 3 DIRECTION SETTINGS
//   FINAL_DIR_X / FINAL_DIR_Y:  1 = positive, -1 = negative, 0 = none
//   Examples:
//     X+ only  →  FINAL_DIR_X =  1,  FINAL_DIR_Y =  0
//     X- only  →  FINAL_DIR_X = -1,  FINAL_DIR_Y =  0
//     Y+ only  →  FINAL_DIR_X =  0,  FINAL_DIR_Y =  1
//     Diagonal →  FINAL_DIR_X =  1,  FINAL_DIR_Y =  1
// ────────────────────────────────────────────────────────────────────
#define FINAL_DIR_X            -1      // ← CHANGE THIS
#define FINAL_DIR_Y            0      // ← CHANGE THIS
#define FINAL_DISTANCE_STEPS   500    // ← CHANGE THIS

// ────────────────────────────────────────────────────────────────────
// MOTION TIMING
// ────────────────────────────────────────────────────────────────────
#define STEP_DELAY_US    500
#define PHASE_PAUSE_MS   2000

// ────────────────────────────────────────────────────────────────────
// POSITION TRACKING
// ────────────────────────────────────────────────────────────────────
long xPos = 0;
long yPos = 0;

// ────────────────────────────────────────────────────────────────────
// LOW-LEVEL STEPPING
// ────────────────────────────────────────────────────────────────────
void stepX(int dir) {
    digitalWrite(X_DIR, dir > 0 ? HIGH : LOW);
    digitalWrite(X_STEP, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(X_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);
    xPos += dir;
}

void stepY(int dir) {
    digitalWrite(Y_DIR, dir > 0 ? HIGH : LOW);
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(Y_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);
    yPos += dir;
}

// ────────────────────────────────────────────────────────────────────
// AXIS MOVES
// ────────────────────────────────────────────────────────────────────
void moveXSteps(long steps) {
    int  dir   = (steps >= 0) ? 1 : -1;
    long count = labs(steps);
    for (long i = 0; i < count; i++) stepX(dir);
}

void moveYSteps(long steps) {
    int  dir   = (steps >= 0) ? 1 : -1;
    long count = labs(steps);
    for (long i = 0; i < count; i++) stepY(dir);
}

// Move to absolute XY using Bresenham (both axes simultaneously)
void moveToXY(long targetX, long targetY) {
    long dx = targetX - xPos;
    long dy = targetY - yPos;
    int  sx = (dx >= 0) ? 1 : -1;
    int  sy = (dy >= 0) ? 1 : -1;
    dx = labs(dx);
    dy = labs(dy);
    if (dx == 0 && dy == 0) return;

    if (dx >= dy) {
        long err = dx / 2;
        for (long i = 0; i < dx; i++) {
            stepX(sx);
            err -= dy;
            if (err < 0) { err += dx; stepY(sy); }
        }
    } else {
        long err = dy / 2;
        for (long i = 0; i < dy; i++) {
            stepY(sy);
            err -= dx;
            if (err < 0) { err += dy; stepX(sx); }
        }
    }
}

// ────────────────────────────────────────────────────────────────────
// PHASE 1: Snake sweep
// ────────────────────────────────────────────────────────────────────
void runSnakeSweep() {
    Serial.println();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println(" PHASE 1: Snake sweep");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    int  direction = 1;
    long yRows     = Y_LENGTH_STEPS / STEP_SIZE_Y + 1;

    for (long row = 0; row < yRows; row++) {
        Serial.print("  Row "); Serial.print(row);
        Serial.print(" (y="); Serial.print(yPos);
        Serial.println(direction > 0 ? ")  going X+" : ")  going X-");
        moveXSteps(direction * X_LENGTH_STEPS);
        if (row < yRows - 1) moveYSteps(STEP_SIZE_Y);
        direction = -direction;
    }

    Serial.print("  Snake done. Position: (");
    Serial.print(xPos); Serial.print(", "); Serial.print(yPos); Serial.println(")");
}

// ────────────────────────────────────────────────────────────────────
// PHASE 2: Return to center of the sweep area
// ────────────────────────────────────────────────────────────────────
void returnToCenter() {
    long centerX = X_LENGTH_STEPS / 2;
    long centerY = Y_LENGTH_STEPS / 2;

    Serial.println();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.print(" PHASE 2: Returning to center (");
    Serial.print(centerX); Serial.print(", "); Serial.print(centerY); Serial.println(")");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    moveToXY(centerX, centerY);

    Serial.print("  At center: (");
    Serial.print(xPos); Serial.print(", "); Serial.print(yPos); Serial.println(")");
}

// ────────────────────────────────────────────────────────────────────
// PHASE 3: Move in the configured direction
// ────────────────────────────────────────────────────────────────────
void moveInSetDirection() {
    Serial.println();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.print(" PHASE 3: Moving — dirX="); Serial.print(FINAL_DIR_X);
    Serial.print("  dirY="); Serial.print(FINAL_DIR_Y);
    Serial.print("  dist="); Serial.print(FINAL_DISTANCE_STEPS); Serial.println(" steps");
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    long targetX = xPos + (long)FINAL_DIR_X * FINAL_DISTANCE_STEPS;
    long targetY = yPos + (long)FINAL_DIR_Y * FINAL_DISTANCE_STEPS;
    moveToXY(targetX, targetY);

    Serial.print("  Final position: (");
    Serial.print(xPos); Serial.print(", "); Serial.print(yPos); Serial.println(")");
}

// ────────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);

    pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
    pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
    digitalWrite(X_STEP, LOW);
    digitalWrite(Y_STEP, LOW);

    Serial.println();
    Serial.println("=========================================");
    Serial.println(" SCAN + DIRECTION  (waiting for trigger)");
    Serial.println("=========================================");
    Serial.print("  Snake area:      "); Serial.print(X_LENGTH_STEPS);
    Serial.print(" x "); Serial.print(Y_LENGTH_STEPS); Serial.println(" steps");
    Serial.print("  Snake row gap:   "); Serial.print(STEP_SIZE_Y); Serial.println(" steps");
    Serial.print("  Final direction: (");
    Serial.print(FINAL_DIR_X); Serial.print(", "); Serial.print(FINAL_DIR_Y); Serial.println(")");
    Serial.print("  Final distance:  "); Serial.print(FINAL_DISTANCE_STEPS); Serial.println(" steps");
    Serial.println();
    Serial.println("Ready. Place nozzle at corner (0,0).");
    Serial.println("Waiting for 'S' from Pi...");
}

// ────────────────────────────────────────────────────────────────────
// LOOP — idles until 'S' is received, then runs the full sequence.
//        Position resets each time so re-triggering always works.
// ────────────────────────────────────────────────────────────────────
void loop() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'S') {
            // Reset tracked position — operator must return nozzle to (0,0)
            // between runs if re-triggering
            xPos = 0;
            yPos = 0;

            Serial.println();
            Serial.println(">>> START received — running sequence <<<");

            runSnakeSweep();
            delay(PHASE_PAUSE_MS);

            returnToCenter();
            delay(PHASE_PAUSE_MS);

            moveInSetDirection();

            Serial.println();
            Serial.println("=== COMPLETE — send 'S' again to re-run ===");
        }
    }
}