# PulseHeartLab — Quick Tutorial

This short, practical guide targets students and teachers. You’ll wire a PPG sensor, run the first sketch, understand the processing pipeline, tune detection, and export data for analysis.

## Requirements

- Arduino Uno (ATmega328P) or compatible
- Analog PPG sensor (e.g., PulseSensor, MAX30102 breakout in analog mode, or generic PPG analog module)
- USB cable and Arduino IDE (1.8+ or 2.x)
- Library installed in `Arduino/libraries/PulseHeartLab`

## Wiring

- Sensor `VCC` → `5V`
- Sensor `GND` → `GND`
- Sensor `OUT` → `A0`

ASCII sketch:

  +5V ---- VCC (sensor)
   GND ---- GND (sensor)
   A0  ---- OUT (sensor)

Tips:
- Keep wires short and stable to reduce motion artifacts.
- Place fingertip gently on the sensor—firm but not tight.

## Install & Quick Start

1) Copy the `PulseHeartLab` folder into `Arduino/libraries/`.
2) Restart Arduino IDE → File → Examples → `PulseHeartLab` → `BasicPlot`.
3) Select board/port, upload, then open Serial Plotter at `115200` baud.

You should see a periodic waveform. Try breathing deeply or raising your hand to notice amplitude changes.

## Minimal Example

```cpp
#include <PulseHeartLab.h>
PulseHeartLab ppg(A0);

void setup() {
  Serial.begin(115200);
  ppg.begin(true, true, 100); // autoStart, autoCalibrate, 100 Hz
}

void loop() {
  int s = ppg.readSignal();
  if (s >= 0) Serial.println(s);
}
```

Notes:
- `begin(autoStart, autoCalibrate, sampleRateHz)`; 100 Hz is a good default.
- `readSignal()` returns a processed envelope suitable for plotting/thresholding.

## How It Works

- Sampling: ~100 Hz via Timer2 ISR (fallback to polling if timers are disabled).
- Calibration (optional): a 256-bin histogram estimates p5/p50/p95 to center and scale the signal.
- Preprocessing: high-pass to remove drift, moving-average smoothing, and slope-sum (SSF) to emphasize upstrokes.
- Detection: adaptive threshold with refractory; IBI→BPM with smoothing and outlier control.
- Notch: optional 50/60 Hz notch with fixed-point biquad when needed.
- SQI: a simple template-based score (0..100) for teaching signal quality.

## Tuning Guide (quick)

- Start with auto-calibration: `begin(true, true, 100)`.
- If ambient noise or mains hum is visible: `ppg.setNotch(50)` or `ppg.setNotch(60)`.
- For very low signals, warm the finger and adjust pressure; recalibrate if needed.
- To visualize thresholds and peaks, use `examples/ThresholdTuning`.

## Examples Tour

- `BasicPlot`: plot the processed signal for learning the waveform.
- `BPMSerial`: print smoothed BPM values; a beat callback can blink an LED.
- `ThresholdTuning`: view threshold/noise/peak levels to understand detection.
- `NotchDemo`: show impact of 50/60 Hz notch on a noisy signal.
- `TemplateDemo`: illustrates the teaching template and alignment.
- `QualityDemo`: prints SQI (0..100) to compare signal quality conditions.
- `TestSignal`: synthetic two-Gaussian PPG—practice without hardware.
- `PhysioZooExport`: record one sample per line (TXT) for external analysis.

## Classroom Activities

- Resting HR: record BPM for 30 s; compute average and discuss variability.
- Exercise HR: 1 min light activity, then measure immediately and after 1 min; discuss recovery.
- Signal Quality: compare loose vs firm finger placement; observe SQI and waveform changes.

## Troubleshooting

- BPM is 0 or intermittent:
  - Check wiring and finger placement; avoid pressing too hard.
  - Try `begin(true, true, 100)` to recalibrate at rest.
- Strong 50/60 Hz line hum:
  - Keep cables short and stable; enable `setNotch(50)` or `setNotch(60)`.
- Flat line or no plot:
  - Ensure Serial Plotter at `115200` baud; verify `Serial.println()` in the loop.

## Export to PhysioZoo / pyPPG

Use `examples/PhysioZooExport` to print one sample per line (TXT). Then, in PhysioZoo Loader:
- Select species (e.g., Mammal), enter Fs (your sample rate), choose channel type (PPG), and units.
- Load the TXT file; the Loader will use your inputs as metadata.

Tip: If you need a headered format for a specific workflow, adapt the example to print a small header before the samples with your desired fields.
