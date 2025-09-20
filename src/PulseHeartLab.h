/**
 * PulseHeartLab (simplified)
 * Beginner-friendly PPG processing for Arduino Uno.
 * Public API focuses on: start/stop sampling, reading raw/processed samples,
 * getting BPM/SQI, and basic configuration (threshold, refractory, notch).
 */
#ifndef PULSEHEARTLAB_H
#define PULSEHEARTLAB_H

#include <Arduino.h>

class PulseHeartLab {
public:
    // Constructor: pass the analog pin (e.g., A0)
    PulseHeartLab(uint8_t analogPin);

    // Initialize library (call in setup).
    // Convenience overloads:
    // - begin(): defaults to Timer2 ISR at 100 Hz, no calibration, not running
    // - begin(autoStart, autoCalibrate, sampleRateHz): Timer2 ISR, optional auto calibration
    // - begin(autoStart, autoCalibrate, sampleRateHz, useTimer): choose Timer2 ISR (true) or polling (false)
    void begin();
    void begin(bool autoStart, bool autoCalibrate, uint16_t sampleRateHz = 100);
    // Overload: choose between timer ISR (default) or polling
    void begin(bool autoStart, bool autoCalibrate, uint16_t sampleRateHz, bool useTimer);

    // Start sampling and processing
    void start();

    // Read raw or calibrated sample from the sensor.
    // Returns -1 if no new sample is available at the current call.
    // If not calibrated: returns ADC reading (0..1023).
    // If calibrated (autoCalibrate=true in begin): returns centered & scaled value (can be negative).
    int readRaw();

    // Read processed/enhanced signal (HP, LP, SSF). For plotting/teaching.
    // Returns -1 if no new sample is available at the current call.
    int readSignal();

    // Smoothed BPM. Returns 0 if not available yet.
    uint16_t getBPM();

    // Threshold/levels for tuning (observability)
    int getThreshold();
    int getPeakLevel();
    int getNoiseLevel();

    // Powerline notch configuration: 0=OFF, 50=50 Hz, 60=60 Hz (requires Fs>=120 Hz)
    void setNotch(uint8_t hz);

    // Signal Quality Index (0..100)
    uint8_t getSQI();

    // Beat callback: invoked outside ISR when a valid beat is detected (passes current BPM)
    void onBeat(void (*cb)(uint16_t bpm));

    // Basic configuration
    void setRefractory(uint16_t ms);
    void setThresholdAuto(bool enabled);

    // Stop sampling/processing
    void stop();

    // Calibration info (baseline average ADC found during autoCalibrate)
    int getBaseline();

    // Exposed for ISR trampoline
    void onSampleISR();

private:
    uint8_t _pin;
    bool _running;
    bool _useTimer; // true: Timer2 ISR; false: polling in loop
    uint16_t _sampleRateHz;
    int _baseline; // baseline ADC value estimated during calibration
    bool _calibrated; // true after successful autoCalibrate
    int16_t _gainQ10; // fixed-point gain (Q10) to scale centered signal
    int _span90;   // p95 - p5 span (ADC units)
    int _deadband; // small band around baseline to allow drift tracking

    // Scheduled sampling (fallback when not using ISR)
    unsigned long _nextSampleMs;

    // Lightweight filters
    int _hp_prev_x;
    int _hp_prev_y;
    static const uint8_t LP_WIN = 5; // short window
    int _lp_buf[5];
    uint8_t _lp_idx;
    long _lp_sum;
    int _lp_prev;

    // SSF (Slope Sum Function) + short-window integration
    static const uint8_t SSF_WIN = 8; // ~80 ms @100 Hz
    int _ssf_buf[SSF_WIN];
    uint8_t _ssf_idx;
    long _ssf_sum;

    // Adaptive detector
    int _proc;            // last processed value
    int _thresh;          // current threshold
    int _peakLevel;
    int _noiseLevel;
    bool _threshAuto;
    uint16_t _refractoryMs;
    unsigned long _lastBeatMs;
    uint16_t _bpm;        // instantaneous bpm
    uint16_t _bpmSmooth;  // smoothed bpm
    uint16_t _ibiEMA;     // IBI EMA for outlier rejection (ms)
    void (*_beatCb)(uint16_t bpm);

    // Internals
    int _applyFilters(int x);
    void _updateDetector(int env, unsigned long tNow);
    void _timerStart();
    void _timerStop();

    // Shared state with ISR
    volatile uint32_t _tick;
    volatile uint32_t _lastReadRawTick;
    volatile uint32_t _lastReadSigTick;
    volatile unsigned long _tMs; // elapsed time in ms (approx) based on Fs
    volatile int _rawADC_isr;    // last raw ADC sample
    volatile int _scaled_isr;    // last centered/scaled sample
    volatile int _proc_isr;      // last processed sample (envelope)
    volatile bool _beatFlag_isr;

    // Notch biquad (Q15) DF2T
    bool _notchEnabled;
    int16_t _nb0, _nb1, _nb2; // b0,b1,b2 in Q15
    int16_t _na1, _na2;       // a1,a2 in Q15 (a0=1)
    int32_t _nz1, _nz2;       // DF2T states
    uint8_t _notchHz;         // 0/50/60

    // Processed-signal history for template
    static const uint8_t HIST_LEN = 64;
    int16_t _hist_env[HIST_LEN];
    uint8_t _hist_idx;

    // Simple pulse template
    static const uint8_t TPL_LEN = 25;
    int8_t _tpl[TPL_LEN];
    bool _tpl_ready;
    uint8_t _sqi; // 0..100

    // Internals
    int _applyNotch(int x);
    void _updateSQIandTemplate();
};

#endif
