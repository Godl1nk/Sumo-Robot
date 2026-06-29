#include <Arduino.h>
#include <SharpIR.h>
#include <QTRSensors.h>

/* =========================
   MOTOR DRIVER (UART)
   SmartDriveDuo-10
========================= */
#define MOTOR_TX 17
#define MOTOR_RX 16
HardwareSerial MotorSerial(2);

/* =========================
   LINE SENSORS (4 corners)
========================= */
#define LINE_FL 27
#define LINE_FR 26
#define LINE_BL 25
#define LINE_BR 33   // confirm GPIO safe

/* =========================
   ENCODERS (Quadrature)
========================= */
#define ENC_L_A 32
#define ENC_L_B 35
#define ENC_R_A 34
#define ENC_R_B 39

volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;

/* =========================
   TOF / IR ENEMY SENSORS
========================= */
#define FRONT_LEFT 14
#define FRONT_CENTER 12
#define FRONT_RIGHT 13
#define SIDE_LEFT 15
#define SIDE_RIGHT 4

/* =========================
   LIBRARY INSTANTIATIONS
========================= */
// Sharp IR configurations
#define SHARP_MODEL SharpIR::GP2Y0A21YK0F // default 10-80cm analog model
const uint8_t DETECTION_THRESHOLD_CM = 50; // detect enemy if closer than 50cm

SharpIR enemyFL(SHARP_MODEL, FRONT_LEFT);
SharpIR enemyFC(SHARP_MODEL, FRONT_CENTER);
SharpIR enemyFR(SHARP_MODEL, FRONT_RIGHT);
SharpIR enemySL(SHARP_MODEL, SIDE_LEFT);
SharpIR enemySR(SHARP_MODEL, SIDE_RIGHT);

// QTR reflectance sensor configurations
QTRSensors qtr;
const uint8_t QTR_SENSOR_COUNT = 4;
uint16_t qtrValues[QTR_SENSOR_COUNT];
const uint16_t QTR_THRESHOLD = 500; // raw readings below this = white line detected
// #define QTR_USE_ANALOG // uncomment if using analog QTR sensors

/* =========================
   MOTOR CONTROL PARAMS
========================= */
int baseSpeed = 180;
int turnSpeed = 160;
int attackSpeed = 255;

/* =========================
   STATE MACHINE
========================= */
enum State {
  SEARCH,
  ATTACK,
  RECOVER,
  LUNGE
};

State currentState = LUNGE; // start with initial lunge

// Software debouncing for line sensors
uint8_t lineTriggerCount[4] = {0, 0, 0, 0};
const uint8_t DEBOUNCE_THRESHOLD = 2; // consecutive loop cycles to confirm edge

// Target tracking memory variables
uint32_t lastSeenTime = 0;
const uint32_t TARGET_LOST_TIMEOUT_MS = 250;
int lastAttackLeftSpeed = 0;
int lastAttackRightSpeed = 0;

// Non-blocking recovery state variables
uint32_t recoveryStartTime = 0;
int recoveryPhase = 0; // 0: inactive, 1: straight escape, 2: spin escape
int escapeLeftSpeed = 0;
int escapeRightSpeed = 0;
int spinLeftSpeed = 0;
int spinRightSpeed = 0;

// Lunge state variables
uint32_t lungeStartTime = 0;
const uint32_t LUNGE_DURATION_MS = 400;

// Spiral search variables
uint32_t searchStartTime = 0;
int searchSpinSign = 1; // 1 = spin right, -1 = spin left
const uint32_t SEARCH_FLIP_MS = 1500; // flip direction every 1.5s

// State name lookup for debug logging
const char* stateNames[] = {"SEARCH", "ATTACK", "RECOVER", "LUNGE"};

void enterState(State newState) {
  Serial.print("[STATE] ");
  Serial.println(stateNames[newState]);

  currentState = newState;
  if (newState == RECOVER) {
    recoveryStartTime = millis();
    recoveryPhase = 1; // start in straight escape phase

    // Capture which sensors triggered the edge
    bool fl = (digitalRead(LINE_FL) == LOW);
    bool fr = (digitalRead(LINE_FR) == LOW);
    bool bl = (digitalRead(LINE_BL) == LOW);
    bool br = (digitalRead(LINE_BR) == LOW);

    if (fl || fr) {
      // Front edge detected -> go backward
      escapeLeftSpeed = -baseSpeed;
      escapeRightSpeed = -baseSpeed;
      if (fl && !fr) {
        spinLeftSpeed = turnSpeed;
        spinRightSpeed = -turnSpeed;
      } else {
        spinLeftSpeed = -turnSpeed;
        spinRightSpeed = turnSpeed;
      }
    } else if (bl || br) {
      // Back edge detected -> go forward
      escapeLeftSpeed = baseSpeed;
      escapeRightSpeed = baseSpeed;
      if (bl && !br) {
        spinLeftSpeed = turnSpeed;
        spinRightSpeed = -turnSpeed;
      } else {
        spinLeftSpeed = -turnSpeed;
        spinRightSpeed = turnSpeed;
      }
    } else {
      escapeLeftSpeed = -baseSpeed;
      escapeRightSpeed = -baseSpeed;
      spinLeftSpeed = turnSpeed;
      spinRightSpeed = -turnSpeed;
    }
  } else {
    recoveryPhase = 0;
  }

  if (newState == LUNGE) {
    lungeStartTime = millis();
  }

  if (newState == SEARCH) {
    searchStartTime = millis();
  }

  // Clear tracking memory when leaving ATTACK state
  if (newState != ATTACK) {
    lastAttackLeftSpeed = 0;
    lastAttackRightSpeed = 0;
  }
}

/* =========================
   MOTOR CONTROL FUNCTION
   (Cytron Simplified Serial)
   Throttled: only writes when
   values change or 50ms elapsed.
========================= */
int lastMotorLeft = 0;
int lastMotorRight = 0;
uint32_t lastMotorWriteTime = 0;
const uint32_t MOTOR_KEEPALIVE_MS = 50;

void setMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Skip write if values unchanged and keepalive not due
  bool changed = (left != lastMotorLeft) || (right != lastMotorRight);
  bool keepalive = (millis() - lastMotorWriteTime >= MOTOR_KEEPALIVE_MS);
  if (!changed && !keepalive) return;

  lastMotorLeft = left;
  lastMotorRight = right;
  lastMotorWriteTime = millis();

  byte leftCmd = 64; // default stop (0 or 64)
  if (left > 0) {
    leftCmd = map(left, 1, 255, 1, 63); // 1 (slowest) to 63 (full forward)
  } else if (left < 0) {
    leftCmd = map(-left, 1, 255, 65, 127); // 65 (slowest) to 127 (full reverse)
  }

  byte rightCmd = 192; // default stop (128 or 192)
  if (right > 0) {
    rightCmd = map(right, 1, 255, 129, 191); // 129 (slowest) to 191 (full forward)
  } else if (right < 0) {
    rightCmd = map(-right, 1, 255, 193, 255); // 193 (slowest) to 255 (full reverse)
  }

  MotorSerial.write(leftCmd);
  MotorSerial.write(rightCmd);
}

/* =========================
   STOP
========================= */
void stopMotor() {
  setMotor(0, 0);
}

/* =========================
   SENSOR READS
 ========================= */
struct EnemySensors {
  bool frontLeft;
  bool frontCenter;
  bool frontRight;
  bool sideLeft;
  bool sideRight;
};

// Returns a bitmask of triggered line sensors (raw value < QTR_THRESHOLD = triggered).
// Bit 0: FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
// Software debounced to filter optical/electrical noise spikes.
uint8_t readLineSensors() {
  uint8_t mask = 0;
  qtr.read(qtrValues);

  for (int i = 0; i < QTR_SENSOR_COUNT; i++) {
    if (qtrValues[i] < QTR_THRESHOLD) {
      if (lineTriggerCount[i] < DEBOUNCE_THRESHOLD) {
        lineTriggerCount[i]++;
      }
    } else {
      lineTriggerCount[i] = 0;
    }

    if (lineTriggerCount[i] >= DEBOUNCE_THRESHOLD) {
      mask |= (1 << i);
    }
  }
  return mask;
}

EnemySensors readEnemySensors() {
  EnemySensors sensors;

  // Read distances in cm using fast non-blocking reads (avoidBurstRead = false)
  int distFL = enemyFL.getDistance(false);
  int distFC = enemyFC.getDistance(false);
  int distFR = enemyFR.getDistance(false);
  int distSL = enemySL.getDistance(false);
  int distSR = enemySR.getDistance(false);

  // Consider detected if distance falls within the detection threshold
  sensors.frontLeft = (distFL > 0 && distFL <= DETECTION_THRESHOLD_CM);
  sensors.frontCenter = (distFC > 0 && distFC <= DETECTION_THRESHOLD_CM);
  sensors.frontRight = (distFR > 0 && distFR <= DETECTION_THRESHOLD_CM);
  sensors.sideLeft = (distSL > 0 && distSL <= DETECTION_THRESHOLD_CM);
  sensors.sideRight = (distSR > 0 && distSR <= DETECTION_THRESHOLD_CM);

  return sensors;
}

bool edgeDetected() {
  return readLineSensors() != 0;
}

bool frontDetected() {
  EnemySensors sensors = readEnemySensors();
  return (sensors.frontCenter || sensors.frontLeft || sensors.frontRight);
}

bool strongFront() {
  return readEnemySensors().frontCenter;
}

bool sideLeftDetected() {
  return readEnemySensors().sideLeft;
}

bool sideRightDetected() {
  return readEnemySensors().sideRight;
}

/* =========================
   ATTACK MODE
========================= */
void attack() {
  EnemySensors sensors = readEnemySensors();
  // Charge straight if center is active, OR if both left and right are active (balanced targets)
  if (sensors.frontCenter || (sensors.frontLeft && sensors.frontRight)) {
    lastAttackLeftSpeed = attackSpeed;
    lastAttackRightSpeed = attackSpeed;
  }
  else if (sensors.frontLeft) {
    lastAttackLeftSpeed = turnSpeed;
    lastAttackRightSpeed = attackSpeed;
  }
  else if (sensors.frontRight) {
    lastAttackLeftSpeed = attackSpeed;
    lastAttackRightSpeed = turnSpeed;
  }
  
  // If we haven't tracked any direction yet, default to driving forward
  if (lastAttackLeftSpeed == 0 && lastAttackRightSpeed == 0) {
    lastAttackLeftSpeed = baseSpeed;
    lastAttackRightSpeed = baseSpeed;
  }

  setMotor(lastAttackLeftSpeed, lastAttackRightSpeed);
}

/* =========================
   SEARCH MODE (SPIRAL)
   Forward-biased spin with
   direction flip every 1.5s.
========================= */
void searchOpponent() {
  EnemySensors sensors = readEnemySensors();

  // Side sensor overrides: spin toward detected enemy
  if (sensors.sideLeft) {
    setMotor(-turnSpeed, turnSpeed); // spin left
    return;
  }
  if (sensors.sideRight) {
    setMotor(turnSpeed, -turnSpeed); // spin right
    return;
  }

  // Flip spin direction every SEARCH_FLIP_MS
  if (millis() - searchStartTime >= SEARCH_FLIP_MS) {
    searchSpinSign = -searchSpinSign;
    searchStartTime = millis();
  }

  // Spiral: forward bias + spin (covers ground instead of spinning in place)
  int innerWheel = 80;   // slower wheel moves forward
  int outerWheel = 180;  // faster wheel drives the turn
  if (searchSpinSign > 0) {
    setMotor(outerWheel, innerWheel);  // spiral right
  } else {
    setMotor(innerWheel, outerWheel);  // spiral left
  }
}

/* =========================
   ENCODER ISR INTERRUPTS
========================= */
void IRAM_ATTR readEncoderLeft() {
  if (digitalRead(ENC_L_B) == HIGH) {
    encoderLeftTicks++;
  } else {
    encoderLeftTicks--;
  }
}

void IRAM_ATTR readEncoderRight() {
  if (digitalRead(ENC_R_B) == HIGH) {
    encoderRightTicks++;
  } else {
    encoderRightTicks--;
  }
}

/* =========================
   SETUP
========================= */
void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] SUMO Robot starting...");

  MotorSerial.begin(19200, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
  stopMotor(); // ensure motors stopped on boot

  // Initialize QTR reflectance sensor array
#ifdef QTR_USE_ANALOG
  qtr.setTypeAnalog();
#else
  qtr.setTypeRC();
#endif
  qtr.setSensorPins((const uint8_t[]){LINE_FL, LINE_FR, LINE_BL, LINE_BR}, QTR_SENSOR_COUNT);

  // Sharp IR sensor pins as inputs
  pinMode(FRONT_LEFT, INPUT);
  pinMode(FRONT_CENTER, INPUT);
  pinMode(FRONT_RIGHT, INPUT);
  pinMode(SIDE_LEFT, INPUT);
  pinMode(SIDE_RIGHT, INPUT);

  // Initialize Quadrature Encoder pins
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  // Attach interrupts to ENC_A pins
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderRight, RISING);

  Serial.println("[BOOT] Waiting 5s (competition delay)...");
  delay(5000); // competition arm delay
  stopMotor(); // guarantee stopped before loop starts
  lungeStartTime = millis(); // start lunge timer
  Serial.println("[BOOT] Ready. LUNGE!");
}

/* =========================
   MAIN LOOP
========================= */
void loop() {
  bool edge = edgeDetected();
  bool enemyFront = frontDetected();

  // State Transitions
  if (edge) {
    // Edge detected always triggers recovery immediately, overriding everything
    if (currentState != RECOVER) {
      enterState(RECOVER);
    }
  } else if (currentState == LUNGE) {
    // During lunge: transition to ATTACK if enemy found, else expire after duration
    if (enemyFront) {
      lastSeenTime = millis();
      enterState(ATTACK);
    } else if (millis() - lungeStartTime >= LUNGE_DURATION_MS) {
      enterState(SEARCH);
    }
  } else if (currentState != RECOVER) {
    // Normal SEARCH/ATTACK transitions
    if (enemyFront) {
      lastSeenTime = millis();
      if (currentState != ATTACK) {
        enterState(ATTACK);
      }
    } else {
      bool holdAttack = (currentState == ATTACK) && 
                         (millis() - lastSeenTime < TARGET_LOST_TIMEOUT_MS) &&
                         !sideLeftDetected() && !sideRightDetected();
      if (!holdAttack) {
        if (currentState != SEARCH) {
          enterState(SEARCH);
        }
      }
    }
  }

  // State Execution
  switch (currentState) {
    case LUNGE:
      setMotor(attackSpeed, attackSpeed); // full speed ahead
      break;

    case RECOVER:
      {
        uint32_t elapsed = millis() - recoveryStartTime;
        if (recoveryPhase == 1) {
          if (elapsed < 150) {
            setMotor(escapeLeftSpeed, escapeRightSpeed);
          } else {
            recoveryPhase = 2;
            recoveryStartTime = millis();
            setMotor(spinLeftSpeed, spinRightSpeed);
          }
        } else if (recoveryPhase == 2) {
          if (elapsed < 200) {
            setMotor(spinLeftSpeed, spinRightSpeed);
          } else {
            enterState(SEARCH);
          }
        }
      }
      break;

    case ATTACK:
      attack();
      break;

    case SEARCH:
      searchOpponent();
      break;
  }

  delay(10); // fast control loop (~100Hz)
}