// ========================================
// XY A4988 Controller (Homing + Move XY)
// ========================================

// ===== X AXIS =====
#define X_DIR 12
#define X_STEP 14
#define X_MIN 18
#define X_MAX 19

// ===== Y AXIS =====
#define Y_DIR 17
#define Y_STEP 16
#define Y_MIN 32
#define Y_MAX 33

// ===== 参数 =====
const int STEP_DELAY = 800;
const int HOMING_DELAY = 1200;
const int RELEASE_DELAY = 1500;

// ===== 坐标范围 =====
const float X_RANGE_CM = 40.0; 
const float Y_RANGE_CM = 40.0;

// ===== 位置 =====
long xPos = 0, xMin = 0, xMax = 0;
long yPos = 0, yMin = 0, yMax = 0;

// ===== 转换比例 =====
float stepsPerCmX = 0;
float stepsPerCmY = 0;

bool homingDone = false;

// ========================================
// 工具函数
// ========================================
bool X_MIN_PRESSED() { return digitalRead(X_MIN) == LOW; }
bool X_MAX_PRESSED() { return digitalRead(X_MAX) == LOW; }
bool Y_MIN_PRESSED() { return digitalRead(Y_MIN) == LOW; }
bool Y_MAX_PRESSED() { return digitalRead(Y_MAX) == LOW; }

// ========================================
// STEP 控制
// ========================================
void stepX(int dir, int delayUs) {
  digitalWrite(X_DIR, (dir > 0));
  digitalWrite(X_STEP, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(X_STEP, LOW);
  delayMicroseconds(delayUs);
  xPos += dir;
}

void stepY(int dir, int delayUs) {
  digitalWrite(Y_DIR, (dir > 0));
  digitalWrite(Y_STEP, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(Y_STEP, LOW);
  delayMicroseconds(delayUs);
  yPos += dir;
}

// ========================================
// 通用移动（单轴）
// ========================================
void moveXSteps(long steps, int delayUs) {
  int dir = (steps >= 0) ? 1 : -1;
  long count = abs(steps);

  for (long i = 0; i < count; i++) {
    if ((dir > 0 && X_MAX_PRESSED()) || (dir < 0 && X_MIN_PRESSED())) {
      Serial.println("X STOP: limit");
      return;
    }
    stepX(dir, delayUs);
  }
}

void moveYSteps(long steps, int delayUs) {
  int dir = (steps >= 0) ? 1 : -1;
  long count = abs(steps);

  for (long i = 0; i < count; i++) {
    if ((dir > 0 && Y_MAX_PRESSED()) || (dir < 0 && Y_MIN_PRESSED())) {
      Serial.println("Y STOP: limit");
      return;
    }
    stepY(dir, delayUs);
  }
}

// ========================================
// Homing X
// ========================================
bool homeX() {
  Serial.println("=== HOMING X ===");

  if (X_MIN_PRESSED()) while (X_MIN_PRESSED()) stepX(1, RELEASE_DELAY);
  if (X_MAX_PRESSED()) while (X_MAX_PRESSED()) stepX(-1, RELEASE_DELAY);

  // 去 MAX
  while (!X_MAX_PRESSED()) stepX(1, HOMING_DELAY);
  long maxPos = xPos;

  while (X_MAX_PRESSED()) stepX(-1, RELEASE_DELAY);

  xPos = 0;

  // 去 MIN
  while (!X_MIN_PRESSED()) stepX(-1, HOMING_DELAY);
  long minPos = xPos;
  long travel = abs(xPos);

  while (X_MIN_PRESSED()) stepX(1, RELEASE_DELAY);

  long center = minPos + travel / 2;

  moveXSteps(center - xPos, STEP_DELAY);

  xPos = 0;
  xMin = -travel / 2;
  xMax =  travel / 2;

  stepsPerCmX = (float)(xMax - xMin) / X_RANGE_CM;

  Serial.print("X range steps: ");
  Serial.println(xMax - xMin);
  Serial.print("Steps/cm X: ");
  Serial.println(stepsPerCmX);

  Serial.println("X DONE");
  return true;
}

// ========================================
// Homing Y
// ========================================
bool homeY() {
  Serial.println("=== HOMING Y ===");

  if (Y_MIN_PRESSED()) while (Y_MIN_PRESSED()) stepY(1, RELEASE_DELAY);
  if (Y_MAX_PRESSED()) while (Y_MAX_PRESSED()) stepY(-1, RELEASE_DELAY);

  // 去 MAX
  while (!Y_MAX_PRESSED()) stepY(1, HOMING_DELAY);
  long maxPos = yPos;

  while (Y_MAX_PRESSED()) stepY(-1, RELEASE_DELAY);

  yPos = 0;

  // 去 MIN
  while (!Y_MIN_PRESSED()) stepY(-1, HOMING_DELAY);
  long minPos = yPos;
  long travel = abs(yPos);

  while (Y_MIN_PRESSED()) stepY(1, RELEASE_DELAY);

  long center = minPos + travel / 2;

  moveYSteps(center - yPos, STEP_DELAY);

  yPos = 0;
  yMin = -travel / 2;
  yMax =  travel / 2;

  stepsPerCmY = (float)(yMax - yMin) / Y_RANGE_CM;

  Serial.print("Y range steps: ");
  Serial.println(yMax - yMin);
  Serial.print("Steps/cm Y: ");
  Serial.println(stepsPerCmY);

  Serial.println("Y DONE");
  return true;
}

// ========================================
// ⭐ XY 同时移动（核心）
// ========================================
void moveToXY(float targetX_cm, float targetY_cm) {

  if (!homingDone) {
    Serial.println("ERROR: Home first");
    return;
  }

  // ===== 限制范围 =====
  if (targetX_cm < -20 || targetX_cm > 20 ||
      targetY_cm < -20 || targetY_cm > 20) {
    Serial.println("ERROR: Out of range (-20~20)");
    return;
  }

  // ===== 转换成 step =====
  long targetX = targetX_cm * stepsPerCmX;
  long targetY = targetY_cm * stepsPerCmY;

  Serial.print("Moving to (cm): ");
  Serial.print(targetX_cm);
  Serial.print(", ");
  Serial.println(targetY_cm);

  while (xPos != targetX || yPos != targetY) {

    if (xPos != targetX) {
      stepX((targetX > xPos) ? 1 : -1, STEP_DELAY);
    }

    if (yPos != targetY) {
      stepY((targetY > yPos) ? 1 : -1, STEP_DELAY);
    }
  }

  Serial.println("Arrived");
}

// ========================================
// Serial 控制
// ========================================
void readSerial() {
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // ===== HOMING =====
    if (cmd == "HOME") {
      homeX();
      homeY();
      homingDone = true;
      Serial.println("ALL HOMED");
    }

    // ===== 查询位置 =====
    else if (cmd == "POS?") {
      Serial.print("X: "); Serial.print(xPos);
      Serial.print(" Y: "); Serial.println(yPos);
    }

    // ===== 输入坐标：x,y =====
    else if (cmd.indexOf(',') > 0) {
      int comma = cmd.indexOf(',');

      long x = cmd.substring(0, comma).toInt();
      long y = cmd.substring(comma + 1).toInt();

      moveToXY(x, y);
    }

    else {
      Serial.println("Invalid cmd");
    }
  }
}

// ========================================
void setup() {
  Serial.begin(115200);

  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_MIN, INPUT_PULLUP);
  pinMode(X_MAX, INPUT_PULLUP);

  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_MIN, INPUT_PULLUP);
  pinMode(Y_MAX, INPUT_PULLUP);

  Serial.println("XY READY");
}

// ========================================
void loop() {
  readSerial();
}