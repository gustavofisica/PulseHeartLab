/*
  BPMSerial.ino — Print smoothed BPM

  Purpose: Prints smoothed BPM to Serial for quick checks.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Monitor at 115200 baud.

  Tip: Keep the hand still; wait a few beats for smoothing.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  sensor.begin(true, true, 100);
}

void loop() {
  // Ensure periodic processing
  sensor.readSignal();
  uint16_t bpm = sensor.getBPM();
  if (bpm > 0) {
    Serial.print("BPM: ");
    Serial.println(bpm);
  }
}
