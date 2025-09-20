/*
  BasicPlot.ino — Plot signal and beat markers

  Purpose: Plots the processed signal for learning the PPG waveform.
  A second channel marks beats for easy visualization.

  Wiring: Sensor OUT->A0, VCC->5V, GND->GND
  IDE: Use Serial Plotter at 115200 baud.

  How to use:
  - Upload, open Serial Plotter, place fingertip gently on the sensor.
  - First column: signal; second column: beat marker (0 or ~800).

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/
#include <PulseHeartLab.h>

PulseHeartLab sensor(A0);
volatile bool beatFlag = false;

void beatCb(uint16_t bpm) {
  beatFlag = true;
}

void setup() {
  Serial.begin(115200);
  sensor.onBeat(beatCb);
  // Auto-calibrate and start at 100 Hz
  sensor.begin(true, true, 100);
}

void loop() {
  int y = sensor.readSignal();
  if (y == -1) return; // wait for next sample

  // Serial Plotter-friendly: value + beat marker
  Serial.print(y);
  Serial.print(",");
  if (beatFlag) { Serial.println(800); beatFlag = false; }
  else { Serial.println(0); }
}
