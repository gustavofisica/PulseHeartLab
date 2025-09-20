/*
  ThresholdTuning.ino — Visualize detection dynamics

  Purpose: Demonstrates SSF + short-window integration and prints
  signal, threshold, peak and noise levels to help tune and understand detection.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Plotter at 115200 baud.

  How to use:
  - Upload, place fingertip gently on the sensor, open Serial Plotter.
  - Observe 4 CSV columns: signal, threshold, peak, noise.

  What to observe:
  - Threshold tracks between noise and peaks; peaks decay over time.
  - Good placement increases peak-to-noise gap and stabilizes detection.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);

void setup() {
  Serial.begin(115200);
  // Enable Timer ISR and auto-calibration at 100 Hz
  sensor.begin(true, true, 100);
}

void loop() {
  int s = sensor.readSignal();
  if (s < 0) return;
  // For Serial Plotter: signal, threshold, peak, noise
  Serial.print(s);
  Serial.print(",");
  Serial.print(sensor.getThreshold());
  Serial.print(",");
  Serial.print(sensor.getPeakLevel());
  Serial.print(",");
  Serial.println(sensor.getNoiseLevel());
}
