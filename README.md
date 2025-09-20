# PulseHeartLab

Beginner-friendly PPG (photoplethysmography) library for Arduino Uno. It provides a minimal, safe API to sample a PPG sensor (e.g., PulseSensor), pre-process the waveform, detect beats, and estimate BPM and signal quality.

Features:
- Timer2-based sampling (default 100 Hz) or polling fallback
- Robust auto-calibration (percentiles) and lightweight filtering (HP, LP, SSF)
- Adaptive threshold beat detection and smoothed BPM
- Optional powerline notch (50/60 Hz)
- Simple template correlation → Signal Quality Index (0..100)
- Examples for plotting, tuning, and synthetic test signals

Quick start:
```cpp
#include <PulseHeartLab.h>
PulseHeartLab ppg(A0);

void setup() {
  Serial.begin(115200);
  // autoStart=true, autoCalibrate=false, Fs=100 Hz
  ppg.begin(true, false, 100);
}

void loop() {
  int env = ppg.readSignal(); // processed signal (for plotting)
  if (env >= 0) {
    Serial.println(env);
  }
}
```

Wiring:
- Sensor OUT → `A0`
- Sensor VCC → `5V`
- Sensor GND → `GND`

API overview:
- `begin(autoStart, autoCalibrate, Fs[, useTimer])`: initialize and optionally start; Timer2 ISR by default
- `start()/stop()`: control sampling pipeline
- `readRaw()`: raw or calibrated sample; returns `-1` if no new sample
- `readSignal()`: processed/enhanced sample; returns `-1` if no new sample
- `getBPM()`: smoothed BPM (0 if not yet available)
- `onBeat(cb)`: register beat callback (called outside ISR)
- `setNotch(0/50/60)`: powerline notch, requires Fs ≥ 120 Hz
- `getSQI()`: signal quality index 0..100

Notes:
- If you pass `autoCalibrate=true`, `readRaw()` returns centered & scaled values (may be negative).
- Without calibration, `readRaw()` returns ADC (0..1023). For Serial Plotter, prefer `readSignal()`.

Examples:
- `BasicPlot`: plot processed signal
- `BPMSerial`: print BPM and event markers
- `ThresholdTuning`: visualize threshold/peak/noise dynamics
- `NotchDemo`: toggle 50/60 Hz notch
- `TemplateDemo`: show template and SQI
- `QualityDemo`: display SQI trend
- `TestSignal`: synthetic PPG generator (two Gaussians) for teaching
- `PhysioZooExport`: record PPG to `.txt` for PhysioZoo/pyPPG

PhysioZoo/pyPPG export:
- PhysioZoo supports WFDB, MAT, and PhysioZoo-specific TXT. You can also import a plain TXT and fill metadata via the PZ Loader.
- The `PhysioZooExport` example records a single-channel time series at a fixed sampling rate and writes a TXT file compatible with PZ Loader. You will be asked in the PZ Loader to confirm: Mammal (e.g., `human`), Fs (e.g., `100`), Channel type (`electrography`) and unit (`volt`/`millivolt`).
- To match the public example path referenced in the docs (Human_example_ppg.txt), keep the format as one sample per line (no header). If you add a header, follow the “Formats supported” tutorial from PhysioZoo.

Documentation:
- See `docs/Tutorial.md` for a student/teacher friendly tutorial and exercises.
- PhysioZoo/pyPPG docs: https://pyppg.readthedocs.io/en/latest/tutorials/PZ_PPG.html and format details: https://docs.physiozoo.com/en/stable/sections/tutorials/pzformats.html

Non‑Medical Disclaimer:
- This library and its examples are intended for educational use in robotics and STEM learning with hobby PPG/heart-rate sensors.
- It is NOT a medical device, does NOT provide diagnostic measurements, and must not be used for medical decisions.
- Results may be affected by motion, ambient light, and sensor limitations; they do not replace professional medical equipment or validated clinical methods.
- For any health-related evaluation, consult qualified healthcare professionals and use approved medical devices and official medical procedures.

License: see repository LICENSE.
