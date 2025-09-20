/*
  NotchDemo.ino — Powerline notch (50/60 Hz)

  Purpose: Demonstrates the effect of enabling a powerline notch.
  Use it only when you clearly see 50/60 Hz hum in the plot.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Plotter at 115200 baud.

  Tip: Try switching between 0 (off), 50 and 60 to compare.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  sensor.begin(true, true, 100);
  // Try 50 or 60 to observe the difference
  sensor.setNotch(50);
}

void loop() {
  int s = sensor.readSignal();
  if (s < 0) return;
  Serial.println(s);
}
