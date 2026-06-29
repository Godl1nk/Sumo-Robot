#include <Arduino.h>
#include <QTRSensors.h>

/* =========================
   LINE SENSORS (4 corners)
========================= */
#define LINE_FL 27
#define LINE_FR 26
#define LINE_BL 25
#define LINE_BR 33

QTRSensors qtr;
const uint8_t QTR_SENSOR_COUNT = 4;
uint16_t qtrValues[QTR_SENSOR_COUNT];

// #define QTR_USE_ANALOG // uncomment if using analog QTR sensors

void setup() {
  Serial.begin(115200);
  Serial.println("[QTR TEST] Starting QTR reflectance sensor test...");

  // Initialize QTR reflectance sensor array
#ifdef QTR_USE_ANALOG
  qtr.setTypeAnalog();
#else
  qtr.setTypeRC();
#endif
  qtr.setSensorPins((const uint8_t[]){LINE_FL, LINE_FR, LINE_BL, LINE_BR}, QTR_SENSOR_COUNT);
}

void loop() {
  // Read raw QTR sensor reflectance values
  qtr.read(qtrValues);

  // Print raw values to Serial Monitor
  // Note: For RC sensors, lower values = lighter surface (e.g. white line).
  //       For Analog sensors, lower values = lighter surface.
  Serial.print("FL: "); Serial.print(qtrValues[0]); Serial.print(" | ");
  Serial.print("FR: "); Serial.print(qtrValues[1]); Serial.print(" | ");
  Serial.print("BL: "); Serial.print(qtrValues[2]); Serial.print(" | ");
  Serial.print("BR: "); Serial.println(qtrValues[3]);

  delay(250);
}
