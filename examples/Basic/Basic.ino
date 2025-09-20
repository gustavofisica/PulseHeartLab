/*
  Basic.ino — Minimal usage

  Purpose: Smallest example using the simplified API.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Plotter at 115200 baud.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  // Use convenience begin: autoStart=true, autoCalibrate=true, 100Hz sample rate
  sensor.begin(true, true, 100);
}

void loop() {
  int y = sensor.readSignal();
  if (y >= 0) {
    Serial.println(y);
  }
}
