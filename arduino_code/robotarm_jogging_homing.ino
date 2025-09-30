#include <Servo.h>

// ============================
// ====== USER SETTINGS =======
// ============================

// --- Servo (optional) ---
int servoMin = 60;   // adjust if you use PCA9685 later
int servoMax = 170;
#define SERVO_PIN 11
Servo myservo;

// --- LED & Endstops (RAMPS default: X=D3, Y=D14, Z=D18) ---
const int ledPin   = 13;
const int endstop_1 = 3;    // X / J1  (NC -> GND)
const int endstop_2 = 14;   // Y / J2  (NC -> GND)
const int endstop_3 = 18;   // Z / J3  (NC -> GND)

// If you DON'T have an endstop wired on an axis yet, set to false
const bool HAS_ENDSTOP_J1 = true;
const bool HAS_ENDSTOP_J2 = true;   // <-- set false if Y switch not wired
const bool HAS_ENDSTOP_J3 = true;   // <-- set false if Z switch not wired

// --- Pin mapping (Arduino Mega + CNC Shield/RAMPS) ---
// J1 = X, J2 = Y, J3 = Z
#define J1_STEP_PIN 54
#define J1_DIR_PIN  55
#define J1_EN_PIN   38

#define J2_STEP_PIN 60
#define J2_DIR_PIN  61
#define J2_EN_PIN   56

#define J3_STEP_PIN 46
#define J3_DIR_PIN  48
#define J3_EN_PIN   62

// --- Motion & homing ---
static const uint8_t STEP_PULSE_US = 30;    // step HIGH width
const uint32_t HOMING_SPEED_SPS = 2000;     // steps/sec during homing
// Homing direction: -1 toward min switch, +1 toward max switch
#define HOME_DIR_J1 +1
#define HOME_DIR_J2 +1
#define HOME_DIR_J3 -1
// Pull-off (move away from switch) after it triggers, in steps
const uint32_t HOMING_PULLOFF_J1 = 800;
const uint32_t HOMING_PULLOFF_J2 = 800;
const uint32_t HOMING_PULLOFF_J3 = 800;

// Safety: maximum steps allowed while approaching the switch (prevents endless run)
const uint32_t HOME_APPROACH_LIMIT_J1 = 200000UL;  // tune to your travel
const uint32_t HOME_APPROACH_LIMIT_J2 = 200000UL;
const uint32_t HOME_APPROACH_LIMIT_J3 = 200000UL;

// --- Steps/mm (for reporting only; does not affect motion directly) ---
float STEPS_PER_MM_J1 = 80.0f;
float STEPS_PER_MM_J2 = 80.0f;
float STEPS_PER_MM_J3 = 400.0f;

// --- Runtime, editable by commands ---
volatile uint32_t steps_per_sec = 1000;   // v#### (shared speed for jogs)
volatile uint32_t step_size     = 200;    // p#### (steps per keypress)

// ============================
// ====== INTERNAL STATE ======
// ============================

struct Axis {
  uint8_t stepPin, dirPin;
  volatile int8_t  dir;          // +1, -1, or 0
  volatile uint32_t targetLeft;  // queued jog steps
  uint32_t lastStepMicros;
  bool invert;

  // machine position tracking
  long posSteps = 0;     // machine position in steps
  bool homed    = false;
} j1, j2, j3;

// ============================
// ========= HELPERS ==========
// ============================

inline uint32_t intervalFromSpeed(uint32_t sps) {
  if (sps < 1) sps = 1;
  return (uint32_t)(1000000UL / sps);
}

inline void driverEnable(bool on) {
  // A4988/DRV8825: EN LOW = enabled
  digitalWrite(J1_EN_PIN, on ? LOW : HIGH);
  digitalWrite(J2_EN_PIN, on ? LOW : HIGH);
  digitalWrite(J3_EN_PIN, on ? LOW : HIGH);
}

inline void setDir(const Axis& ax, int8_t d) {
  bool level = (d >= 0);
  if (ax.invert) level = !level;
  digitalWrite(ax.dirPin, level ? HIGH : LOW);
}

inline void pulseStep(const Axis& ax) {
  digitalWrite(ax.stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(ax.stepPin, LOW);
}

inline void stepAndAccount(Axis& ax) {
  pulseStep(ax);
  ax.posSteps += (ax.dir >= 0 ? +1 : -1);
}

inline bool endstopTriggered(int pin) {
  // With INPUT_PULLUP + NC wiring: pressed == LOW(0)
  return digitalRead(pin) == LOW;
}

void setupAxis(Axis& ax, uint8_t stepPin, uint8_t dirPin, bool invert=false) {
  ax.stepPin = stepPin;
  ax.dirPin  = dirPin;
  ax.dir = 0;
  ax.targetLeft = 0;
  ax.lastStepMicros = micros();
  ax.invert = invert;
  pinMode(ax.stepPin, OUTPUT);
  pinMode(ax.dirPin,  OUTPUT);
  digitalWrite(ax.stepPin, LOW);
}

void cancelAll() {
  j1.targetLeft = j2.targetLeft = j3.targetLeft = 0;
  j1.dir = j2.dir = j3.dir = 0;
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH); delay(200);
    digitalWrite(ledPin, LOW);  delay(200);
  }
}

void angleServo(int angle) {
  myservo.write(angle);
  delay(300);
}

void reportPos() {
  auto mm = [](long steps, float spmm){ return steps / spmm; };
  Serial.print("MPos (steps): X="); Serial.print(j1.posSteps);
  Serial.print(" Y="); Serial.print(j2.posSteps);
  Serial.print(" Z="); Serial.print(j3.posSteps);
  Serial.print("   |   MPos (mm): X="); Serial.print(mm(j1.posSteps, STEPS_PER_MM_J1), 3);
  Serial.print(" Y="); Serial.print(mm(j2.posSteps, STEPS_PER_MM_J2), 3);
  Serial.print(" Z="); Serial.println(mm(j3.posSteps, STEPS_PER_MM_J3), 3);
}

void printHelp() {
  Serial.println(F("=== Commands ==="));
  Serial.println(F(" Jog: w/s=X±, d/a=Y±, c/z=Z±   (queues step_size steps at steps_per_sec)"));
  Serial.println(F(" v####  -> set speed (steps/sec), e.g. v1500"));
  Serial.println(F(" p####  -> set step size (steps), e.g. p400"));
  Serial.println(F(" E/e    -> enable/disable drivers"));
  Serial.println(F(" x      -> cancel all queued motion"));
  Serial.println(F(" pm1:N  -> blink LED N times"));
  Serial.println(F(" servo:ANGLE -> move servo to ANGLE (0-180)"));
  Serial.println(F(" home, homeX, homeY, homeZ -> homing (with safety & diagnostics)"));
  Serial.println(F(" G92 [X0] [Y0] [Z0] -> set current position as zero for specified axes"));
  Serial.println(F(" M114  -> report machine position"));
}

// ============================
// ======= HOMING LOGIC =======
// ============================

bool homeOne(Axis& ax, const char* name,
             int endstopPin, bool hasEndstop,
             int8_t homeDir, uint32_t feed_sps,
             uint32_t pulloff_steps, uint32_t approach_limit_steps)
{
  Serial.print("[HOME "); Serial.print(name); Serial.print("] ");
  if (!hasEndstop) {
    Serial.println("SKIPPED (HAS_ENDSTOP=false).");
    ax.homed = false;  // not referenced
    return false;
  }

  // Quick status
  int initial = digitalRead(endstopPin);
  Serial.print("endstop@"); Serial.print(endstopPin);
  Serial.print(" initial="); Serial.print(initial);
  Serial.print(" (expect 1 = not pressed, 0 = pressed). ");
  if (initial == LOW) Serial.print(" [PRESSED at start] ");
  Serial.println();

  uint32_t dt = intervalFromSpeed(feed_sps);

  // If already pressed at start, back off first so we can approach cleanly
  if (endstopTriggered(endstopPin)) {
    Serial.print("[HOME "); Serial.print(name); Serial.println("] Already pressed -> backing off...");
    ax.dir = -homeDir; setDir(ax, -homeDir);
    // back off up to 2 * pulloff to ensure release
    for (uint32_t i = 0; i < pulloff_steps * 2; i++) {
      stepAndAccount(ax);
      delayMicroseconds(dt);
      if (!endstopTriggered(endstopPin)) break;
    }
    delay(5); // settle
    if (endstopTriggered(endstopPin)) {
      Serial.print("[HOME "); Serial.print(name); Serial.println("] ERROR: still pressed after backoff.");
      return false;
    }
  }

  // Approach toward switch
  Serial.print("[HOME "); Serial.print(name); Serial.println("] Approaching...");
  ax.dir = homeDir; setDir(ax, homeDir);

  uint8_t lowCount = 0;
  uint32_t steps = 0, debounced_at = 0;
  while (lowCount < 3) {
    // approach limit guard
    if (steps >= approach_limit_steps) {
      Serial.print("[HOME "); Serial.print(name); Serial.println("] FAIL: approach limit reached (no trigger).");
      return false;
    }
    // step
    stepAndAccount(ax); steps++;
    delayMicroseconds(dt);

    if (endstopTriggered(endstopPin)) { lowCount++; if (debounced_at == 0) debounced_at = steps; }
    else lowCount = 0;
  }
  Serial.print("[HOME "); Serial.print(name); Serial.print("] TRIGGERED at step "); Serial.print(debounced_at); Serial.println(".");

  // Pull off (away from switch)
  Serial.print("[HOME "); Serial.print(name); Serial.println("] Pull-off...");
  ax.dir = -homeDir; setDir(ax, -homeDir);
  for (uint32_t i = 0; i < pulloff_steps; i++) {
    stepAndAccount(ax);
    delayMicroseconds(dt);
  }

  // Define machine zero at pull-off point
  ax.posSteps = 0;
  ax.homed = true;
  Serial.print("[HOME "); Serial.print(name); Serial.println("] DONE. Zero set at pull-off.");
  return true;
}

void SetHome() {
  Serial.println("Homing all axes (X -> Y -> Z)...");
  bool okX = homeOne(j1, "X", endstop_1, HAS_ENDSTOP_J1, HOME_DIR_J1, HOMING_SPEED_SPS, HOMING_PULLOFF_J1, HOME_APPROACH_LIMIT_J1);
  bool okY = homeOne(j2, "Y", endstop_2, HAS_ENDSTOP_J2, HOME_DIR_J2, HOMING_SPEED_SPS, HOMING_PULLOFF_J2, HOME_APPROACH_LIMIT_J2);
  bool okZ = homeOne(j3, "Z", endstop_3, HAS_ENDSTOP_J3, HOME_DIR_J3, HOMING_SPEED_SPS, HOMING_PULLOFF_J3, HOME_APPROACH_LIMIT_J3);

  Serial.print("Summary: X="); Serial.print(okX ? "OK" : "SKIP/FAIL");
  Serial.print(" Y="); Serial.print(okY ? "OK" : "SKIP/FAIL");
  Serial.print(" Z="); Serial.println(okZ ? "OK" : "SKIP/FAIL");

  if (!(okX && okY && okZ)) {
    Serial.println("NOTE: One or more axes did not complete homing. Check wiring/pins and HAS_ENDSTOP flags.");
  }
  reportPos();
}

// ============================
// ===== COMMAND HANDLER ======
// ============================

void handleCommand(const String& cmd) {
  if (!cmd.length()) return;

  if (cmd == "home")  { SetHome(); return; }
  if (cmd == "homeX") { homeOne(j1, "X", endstop_1, HAS_ENDSTOP_J1, HOME_DIR_J1, HOMING_SPEED_SPS, HOMING_PULLOFF_J1, HOME_APPROACH_LIMIT_J1); reportPos(); return; }
  if (cmd == "homeY") { homeOne(j2, "Y", endstop_2, HAS_ENDSTOP_J2, HOME_DIR_J2, HOMING_SPEED_SPS, HOMING_PULLOFF_J2, HOME_APPROACH_LIMIT_J2); reportPos(); return; }
  if (cmd == "homeZ") { homeOne(j3, "Z", endstop_3, HAS_ENDSTOP_J3, HOME_DIR_J3, HOMING_SPEED_SPS, HOMING_PULLOFF_J3, HOME_APPROACH_LIMIT_J3); reportPos(); return; }

  if (cmd.startsWith("G92")) {
    // Minimal G92: set current position as zero for any axes mentioned
    if (cmd.indexOf('X') >= 0) j1.posSteps = 0;
    if (cmd.indexOf('Y') >= 0) j2.posSteps = 0;
    if (cmd.indexOf('Z') >= 0) j3.posSteps = 0;
    Serial.println
  }
}
/*
=== Commands ===
 Jog: w/s=X±, d/a=Y±, c/z=Z±   (queues step_size steps at steps_per_sec)
 v####  -> set speed (steps/sec), e.g. v1500
 p####  -> set step size (steps), e.g. p400
 E/e    -> enable/disable drivers
 x      -> cancel all queued motion
 pm1:N  -> blink LED N times
 servo:ANGLE -> move servo to ANGLE (0-180)
 home, homeX, homeY, homeZ -> homing (with safety & diagnostics)
 G92 [X0] [Y0] [Z0] -> set current position as zero for specified axes
 M114  -> report machine position*/