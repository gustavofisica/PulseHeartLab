/*
  TestSignal.ino — Synthetic PPG (two Gaussians)

  Purpose: Practice plotting/analysis without hardware using a simple
  two-Gaussian PPG model (includes a small dicrotic notch by design choices).

  Output: One integer per line (0..1023) for Serial Plotter at 115200 baud.

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/

#include <Arduino.h>
#include <math.h>

const float A1_ = 1.0;
const float mu1 = 0.3;
const float sigma1 = 0.05;

const float A2_ = 0.5;
const float mu2 = 0.6;
const float sigma2 = 0.08;

const int sampleRate = 50;      // Hz
float t = 0.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Normalized time (0..1)
  float tn = fmod(t, 1.0);

  // PPG pulse equation: sum of two Gaussians
  float signal = A1_ * exp(-pow(tn - mu1, 2) / (2.0 * pow(sigma1, 2))) +
                 A2_ * exp(-pow(tn - mu2, 2) / (2.0 * pow(sigma2, 2)));

  // Scale to 10-bit (0..1023)
  int output = (int)(signal * 512.0);
  output = constrain(output, 0, 1023);

  // Send to Serial
  Serial.println(output);

  // Next point
  delay(1000 / sampleRate);
  t += 1.0 / sampleRate;
}
