#include <Arduino.h>

/* =========================
   MOTOR DRIVER (UART)
   SmartDriveDuo-10
========================= */
#define MOTOR_TX 17
#define MOTOR_RX 16
HardwareSerial MotorSerial(2);

void setMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

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

void stopMotor() {
  setMotor(0, 0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("[MOTOR TEST] Starting Cytron MDDS10 Simplified Serial test...");

  MotorSerial.begin(19200, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
  stopMotor();
  delay(1000);
}

void loop() {
  Serial.println("Moving: FORWARD (speed: 150)");
  setMotor(150, 150);
  delay(1000);

  Serial.println("Moving: STOP");
  stopMotor();
  delay(1000);

  Serial.println("Moving: REVERSE (speed: -150)");
  setMotor(-150, -150);
  delay(1000);

  Serial.println("Moving: STOP");
  stopMotor();
  delay(1000);

  Serial.println("Moving: SPIN LEFT");
  setMotor(-150, 150);
  delay(1000);

  Serial.println("Moving: SPIN RIGHT");
  setMotor(150, -150);
  delay(1000);

  Serial.println("Moving: STOP");
  stopMotor();
  delay(2000);
}
