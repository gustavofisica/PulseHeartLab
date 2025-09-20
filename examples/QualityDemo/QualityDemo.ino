/*
  QualityDemo.ino — Print BPM and SQI

  Purpose: Displays BPM and SQI for quick quality checks.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Monitor at 115200 baud.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  sensor.begin(true, true, 100);
}

void loop() {
  sensor.readSignal();
  static unsigned long t0=0; unsigned long now=millis();
  if (now - t0 >= 200) { // print every 200 ms
    t0 = now;
    Serial.print("BPM: "); Serial.print(sensor.getBPM());
    Serial.print("  SQI: "); Serial.println(sensor.getSQI());
  }
}
