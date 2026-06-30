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
   CONFIGURATION PARAMETERS
========================= */
// Sensor settings
int sharpModel = SharpIR::GP2Y0A21YK0F;       // default GP2Y0A21YK0F (10-80cm) analog model
int enemyDetectionThresholdCm = 50;         // max distance in cm to trigger enemy detection
int qtrLineThreshold = 500;                 // raw reflectance below this = white line detected
int qtrDebounceThreshold = 2;               // consecutive loop cycles to confirm edge detection

// Behavior timings
uint32_t enemyLostTimeoutMs = 250;          // short-term memory to keep charging on sensor dropout
uint32_t lungeDurationMs = 400;             // initial startup lunge duration
uint32_t searchFlipDurationMs = 1500;       // how often search direction is inverted
uint32_t motorKeepaliveMs = 50;             // keepalive interval for serial motor command throttle

// Speed settings (0 to 255)
int motorBaseSpeed = 180;
int motorTurnSpeed = 160;
int motorAttackSpeed = 255;

// Encoder navigation settings
long recoveryBackupTicks = 350;             // target ticks to back up during straight escape
long recoverySpinTicks = 250;               // target ticks to spin during recovery turn
uint32_t stuckCheckIntervalMs = 300;        // interval in ms to check if wheels are stuck
long stuckMinTicks = 15;                    // minimum ticks moved to not be considered stuck
uint32_t recoveryTimeoutMs = 600;           // safety timeout in ms for recovery phases
float lungeKP = 0.5;                        // proportional steering correction for lunge

/* =========================
   LIBRARY INSTANTIATIONS
========================= */
SharpIR enemyFL(sharpModel, FRONT_LEFT);
SharpIR enemyFC(sharpModel, FRONT_CENTER);
SharpIR enemyFR(sharpModel, FRONT_RIGHT);
SharpIR enemySL(sharpModel, SIDE_LEFT);
SharpIR enemySR(sharpModel, SIDE_RIGHT);

// QTR reflectance sensor configurations
QTRSensors qtr;
const uint8_t QTR_SENSOR_COUNT = 4;
uint16_t qtrValues[QTR_SENSOR_COUNT];
// #define QTR_USE_ANALOG // uncomment if using analog QTR sensors

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

// Target tracking memory variables
uint32_t lastSeenTime = 0;
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

// Spiral search variables
uint32_t searchStartTime = 0;
int searchSpinSign = 1; // 1 = spin right, -1 = spin left

// Encoder tracking variables
long recoveryStartLeftTicks = 0;
long recoveryStartRightTicks = 0;
long lungeStartLeftTicks = 0;
long lungeStartRightTicks = 0;
uint32_t lastStuckCheckTime = 0;
long lastStuckLeftTicks = 0;
long lastStuckRightTicks = 0;

// State name lookup for debug logging
const char* stateNames[] = {"SEARCH", "ATTACK", "RECOVER", "LUNGE"};

void enterState(State newState) {
  Serial.print("[STATE] ");
  Serial.println(stateNames[newState]);

  currentState = newState;

  // Reset stuck check parameters on state entry
  lastStuckCheckTime = millis();
  lastStuckLeftTicks = encoderLeftTicks;
  lastStuckRightTicks = encoderRightTicks;

  if (newState == RECOVER) {
    recoveryStartTime = millis();
    recoveryPhase = 1; // start in straight escape phase
    recoveryStartLeftTicks = encoderLeftTicks;
    recoveryStartRightTicks = encoderRightTicks;

    // Capture which sensors triggered the edge (using QTR raw values and threshold)
    bool fl = (qtrValues[0] < qtrLineThreshold);
    bool fr = (qtrValues[1] < qtrLineThreshold);
    bool bl = (qtrValues[2] < qtrLineThreshold);
    bool br = (qtrValues[3] < qtrLineThreshold);

    if (fl || fr) {
      // Front edge detected -> go backward
      escapeLeftSpeed = -motorBaseSpeed;
      escapeRightSpeed = -motorBaseSpeed;
      if (fl && !fr) {
        spinLeftSpeed = motorTurnSpeed;
        spinRightSpeed = -motorTurnSpeed;
      } else {
        spinLeftSpeed = -motorTurnSpeed;
        spinRightSpeed = motorTurnSpeed;
      }
    } else if (bl || br) {
      // Back edge detected -> go forward
      escapeLeftSpeed = motorBaseSpeed;
      escapeRightSpeed = motorBaseSpeed;
      if (bl && !br) {
        spinLeftSpeed = motorTurnSpeed;
        spinRightSpeed = -motorTurnSpeed;
      } else {
        spinLeftSpeed = -motorTurnSpeed;
        spinRightSpeed = motorTurnSpeed;
      }
    } else {
      escapeLeftSpeed = -motorBaseSpeed;
      escapeRightSpeed = -motorBaseSpeed;
      spinLeftSpeed = motorTurnSpeed;
      spinRightSpeed = -motorTurnSpeed;
    }
  } else {
    recoveryPhase = 0;
  }

  if (newState == LUNGE) {
    lungeStartTime = millis();
    lungeStartLeftTicks = encoderLeftTicks;
    lungeStartRightTicks = encoderRightTicks;
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

void setMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Skip write if values unchanged and keepalive not due
  bool changed = (left != lastMotorLeft) || (right != lastMotorRight);
  bool keepalive = (millis() - lastMotorWriteTime >= motorKeepaliveMs);
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

// Returns a bitmask of triggered line sensors (raw value < qtrLineThreshold = triggered).
// Bit 0: FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
// Software debounced to filter optical/electrical noise spikes.
uint8_t readLineSensors() {
  uint8_t mask = 0;
  qtr.read(qtrValues);

  for (int i = 0; i < QTR_SENSOR_COUNT; i++) {
    if (qtrValues[i] < qtrLineThreshold) {
      if (lineTriggerCount[i] < qtrDebounceThreshold) {
        lineTriggerCount[i]++;
      }
    } else {
      lineTriggerCount[i] = 0;
    }

    if (lineTriggerCount[i] >= qtrDebounceThreshold) {
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
  sensors.frontLeft = (distFL > 0 && distFL <= enemyDetectionThresholdCm);
  sensors.frontCenter = (distFC > 0 && distFC <= enemyDetectionThresholdCm);
  sensors.frontRight = (distFR > 0 && distFR <= enemyDetectionThresholdCm);
  sensors.sideLeft = (distSL > 0 && distSL <= enemyDetectionThresholdCm);
  sensors.sideRight = (distSR > 0 && distSR <= enemyDetectionThresholdCm);

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
    lastAttackLeftSpeed = motorAttackSpeed;
    lastAttackRightSpeed = motorAttackSpeed;
  }
  else if (sensors.frontLeft) {
    lastAttackLeftSpeed = motorTurnSpeed;
    lastAttackRightSpeed = motorAttackSpeed;
  }
  else if (sensors.frontRight) {
    lastAttackLeftSpeed = motorAttackSpeed;
    lastAttackRightSpeed = motorTurnSpeed;
  }
  
  // If we haven't tracked any direction yet, default to driving forward
  if (lastAttackLeftSpeed == 0 && lastAttackRightSpeed == 0) {
    lastAttackLeftSpeed = motorBaseSpeed;
    lastAttackRightSpeed = motorBaseSpeed;
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
    setMotor(-motorTurnSpeed, motorTurnSpeed); // spin left
    return;
  }
  if (sensors.sideRight) {
    setMotor(motorTurnSpeed, -motorTurnSpeed); // spin right
    return;
  }

  // Flip spin direction every searchFlipDurationMs
  if (millis() - searchStartTime >= searchFlipDurationMs) {
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
    } else if (millis() - lungeStartTime >= lungeDurationMs) {
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
                         (millis() - lastSeenTime < enemyLostTimeoutMs) &&
                         !sideLeftDetected() && !sideRightDetected();
      if (!holdAttack) {
        if (currentState != SEARCH) {
          enterState(SEARCH);
        }
      }
    }
  }

  // Stuck Detection (Only in SEARCH or LUNGE states)
  if (currentState == SEARCH || currentState == LUNGE) {
    if (millis() - lastStuckCheckTime >= stuckCheckIntervalMs) {
      long deltaL = abs(encoderLeftTicks - lastStuckLeftTicks);
      long deltaR = abs(encoderRightTicks - lastStuckRightTicks);
      
      // If we commanded movement but encoders barely moved, we are stuck
      bool stuck = (deltaL < stuckMinTicks && deltaR < stuckMinTicks);
      
      lastStuckLeftTicks = encoderLeftTicks;
      lastStuckRightTicks = encoderRightTicks;
      lastStuckCheckTime = millis();
      
      if (stuck) {
        Serial.println("[STUCK] Robot is stuck! Triggering recovery...");
        enterState(RECOVER);
      }
    }
  }

  // State Execution
  switch (currentState) {
    case LUNGE:
      {
        // Closed-loop straight-line correction using encoders
        long diffL = encoderLeftTicks - lungeStartLeftTicks;
        long diffR = encoderRightTicks - lungeStartRightTicks;
        long error = diffL - diffR; // positive if left is ahead
        
        int correction = (int)(error * lungeKP);
        int leftSpeed = motorAttackSpeed - correction;
        int rightSpeed = motorAttackSpeed + correction;
        
        setMotor(leftSpeed, rightSpeed);
      }
      break;

    case RECOVER:
      {
        uint32_t elapsed = millis() - recoveryStartTime;
        long distL = abs(encoderLeftTicks - recoveryStartLeftTicks);
        long distR = abs(encoderRightTicks - recoveryStartRightTicks);
        long avgDist = (distL + distR) / 2;

        if (recoveryPhase == 1) {
          // Phase 1: Straight escape
          if (avgDist < recoveryBackupTicks && elapsed < recoveryTimeoutMs) {
            setMotor(escapeLeftSpeed, escapeRightSpeed);
          } else {
            // Transition to Phase 2 (Spin escape)
            recoveryPhase = 2;
            recoveryStartTime = millis();
            recoveryStartLeftTicks = encoderLeftTicks;
            recoveryStartRightTicks = encoderRightTicks;
            setMotor(spinLeftSpeed, spinRightSpeed);
          }
        } else if (recoveryPhase == 2) {
          // Phase 2: Spin escape
          if (avgDist < recoverySpinTicks && elapsed < recoveryTimeoutMs) {
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