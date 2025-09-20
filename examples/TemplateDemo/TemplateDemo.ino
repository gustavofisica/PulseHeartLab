/*
  TemplateDemo.ino — Simple template and SQI

  Purpose: Shows a teaching-oriented template and the SQI (0..100).

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Plotter at 115200 baud.

  Tip: Compare SQI with loose vs firm finger placement and motion.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  sensor.begin(true, true, 100);
}

void loop() {
  int s = sensor.readSignal();
  if (s < 0) return;
  // Plot signal and SQI to explore quality
  Serial.print(s);
  Serial.print(",");
  Serial.println(sensor.getSQI());
}
