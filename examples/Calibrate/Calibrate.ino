/*
  Calibrate.ino — Inspect baseline and raw values

  Purpose: Run auto-calibration (no auto-start), print baseline, then start
  and print raw/calibrated values for inspection.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Monitor at 115200 baud.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  // Auto-calibrate for ~1.5s, do not auto-start
  sensor.begin(false, true, 100);
  Serial.print("Baseline ADC: ");
  Serial.println(sensor.getBaseline());
  // now start reading
  sensor.start();
}

void loop() {
  int raw = sensor.readRaw();
  if (raw >= 0) {
    Serial.print("Raw: ");
    Serial.println(raw);
  }
  delay(50);
}
