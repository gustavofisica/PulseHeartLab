/*
  PhysioZooExport.ino — Export TXT for PhysioZoo/pyPPG
  
  Purpose: Record a single-channel series (one integer per line) compatible
  with PhysioZoo's PZ Loader. Metadata is entered in the Loader UI.
  
  How to use:
  - Wire your PPG sensor to A0, 5V, GND.
  - Set SAMPLE_RATE_HZ and DURATION_SEC below.
  - Upload, open Serial Monitor at 115200. Data will be printed as plain lines.
  - To save to a file: use Arduino IDE's "Capture to file" or redirect the serial log.
  - In PhysioZoo: File -> Open data file -> select your .txt. PZ Loader will ask for:
      Mammal: human,  Fs: (e.g.) 100,  Channel type: electrography,  Unit: volt/millivolt.

  Optional: If you have an SD shield, set SAVE_TO_SD=true to write /ppg_export.txt

  Non‑Medical: Educational use only. Not a diagnostic tool.
*/

#include <Arduino.h>
#include <PulseHeartLab.h>

// Config
const uint16_t SAMPLE_RATE_HZ = 100;   // target sampling rate
const uint16_t DURATION_SEC   = 30;    // how long to record
const bool SAVE_TO_SD = false;         // requires SD shield and library

// SD optional
#if defined(ARDUINO) && (SAVE_TO_SD)
  #include <SPI.h>
  #include <SD.h>
  File logFile;
  const int SD_CS_PIN = 10; // adjust if needed
#endif

PulseHeartLab ppg(A0);
unsigned long startMs;
unsigned long nextMs;
unsigned long endMs;

void setup() {
  Serial.begin(115200);
  ppg.begin(true, true, SAMPLE_RATE_HZ); // start + auto-calibrate

#if defined(ARDUINO) && (SAVE_TO_SD)
  if (SD.begin(SD_CS_PIN)) {
    logFile = SD.open("/ppg_export.txt", FILE_WRITE);
    if (logFile) {
      logFile.println("# PhysioZoo TXT export - one sample per line");
      logFile.println("# Mammal=human, Fs=" + String(SAMPLE_RATE_HZ));
    }
  }
#endif

  startMs = millis();
  endMs = startMs + (unsigned long)DURATION_SEC * 1000UL;
  nextMs = startMs;
}

void loop() {
  unsigned long now = millis();
  if (now >= endMs) {
#if defined(ARDUINO) && (SAVE_TO_SD)
    if (logFile) { logFile.flush(); logFile.close(); }
#endif
    // Stop after recording duration
    while (1) { /* done */ }
  }

  // Keep consistent pacing
  if (now < nextMs) return;
  nextMs += (1000UL / SAMPLE_RATE_HZ);

  // Read processed signal; if unavailable, try raw
  int s = ppg.readSignal();
  if (s < 0) s = ppg.readRaw();
  if (s < 0) return; // no new sample yet

  // Print one sample per line
  Serial.println(s);

#if defined(ARDUINO) && (SAVE_TO_SD)
  if (logFile) logFile.println(s);
#endif
}
