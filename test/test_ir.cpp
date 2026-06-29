#include <Arduino.h>
#include <SharpIR.h>

/* =========================
   TOF / IR ENEMY SENSORS
========================= */
#define FRONT_LEFT 14
#define FRONT_CENTER 12
#define FRONT_RIGHT 13
#define SIDE_LEFT 15
#define SIDE_RIGHT 4

// Sharp IR Configurations
#define SHARP_MODEL SharpIR::GP2Y0A21YK0F // default GP2Y0A21YK0F model

SharpIR enemyFL(SHARP_MODEL, FRONT_LEFT);
SharpIR enemyFC(SHARP_MODEL, FRONT_CENTER);
SharpIR enemyFR(SHARP_MODEL, FRONT_RIGHT);
SharpIR enemySL(SHARP_MODEL, SIDE_LEFT);
SharpIR enemySR(SHARP_MODEL, SIDE_RIGHT);

void setup() {
  Serial.begin(115200);
  Serial.println("[IR TEST] Starting Sharp IR distance sensor test...");
  
  pinMode(FRONT_LEFT, INPUT);
  pinMode(FRONT_CENTER, INPUT);
  pinMode(FRONT_RIGHT, INPUT);
  pinMode(SIDE_LEFT, INPUT);
  pinMode(SIDE_RIGHT, INPUT);
}

void loop() {
  // Read distance in cm (using fast non-blocking reads)
  int distFL = enemyFL.getDistance(false);
  int distFC = enemyFC.getDistance(false);
  int distFR = enemyFR.getDistance(false);
  int distSL = enemySL.getDistance(false);
  int distSR = enemySR.getDistance(false);

  // Print raw values to Serial Monitor
  Serial.print("FL: "); Serial.print(distFL); Serial.print(" cm | ");
  Serial.print("FC: "); Serial.print(distFC); Serial.print(" cm | ");
  Serial.print("FR: "); Serial.print(distFR); Serial.print(" cm | ");
  Serial.print("SL: "); Serial.print(distSL); Serial.print(" cm | ");
  Serial.print("SR: "); Serial.print(distSR); Serial.println(" cm");

  delay(250);
}
