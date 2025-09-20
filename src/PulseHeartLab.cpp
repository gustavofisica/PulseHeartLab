/* Minimal PulseHeartLab implementation */

#include <Arduino.h>
#include "PulseHeartLab.h"
extern "C" {
  #include <avr/interrupt.h>
}

static PulseHeartLab* _phl_instance = nullptr; // ISR trampoline target

PulseHeartLab::PulseHeartLab(uint8_t analogPin)
  : _pin(analogPin), _running(false), _useTimer(true), _sampleRateHz(100), _baseline(0), _calibrated(false), _gainQ10(1024), _span90(0), _deadband(0),
    _nextSampleMs(0), _hp_prev_x(0), _hp_prev_y(0), _lp_idx(0), _lp_sum(0), _lp_prev(0), _ssf_idx(0), _ssf_sum(0), _proc(0), _thresh(50), _peakLevel(100), _noiseLevel(20),
    _threshAuto(true), _refractoryMs(250), _lastBeatMs(0), _bpm(0), _bpmSmooth(0), _ibiEMA(0), _beatCb(nullptr),
    _notchEnabled(false), _nb0(0), _nb1(0), _nb2(0), _na1(0), _na2(0), _nz1(0), _nz2(0), _notchHz(0),
    _hist_idx(0), _tpl_ready(false), _sqi(0)
{
}

void PulseHeartLab::begin() {
  // Reset internal state to defaults (100 Hz Timer2 ISR, not running)
  pinMode(_pin, INPUT);
  _sampleRateHz = 100;
  _useTimer = true;
  _baseline = 0;
  _calibrated = false;
  _gainQ10 = 1024; // 1.0 in Q10
  _span90 = 0;
  _deadband = 0;
  _nextSampleMs = millis();
  _hp_prev_x = _hp_prev_y = 0;
  _lp_sum = 0; _lp_idx = 0; _lp_prev = 0; _ssf_sum = 0; _ssf_idx = 0; for (uint8_t i = 0; i < SSF_WIN; ++i) _ssf_buf[i] = 0;
  for (uint8_t i = 0; i < LP_WIN; ++i) _lp_buf[i] = 0;
  _proc = 0; _thresh = 50; _peakLevel = 100; _noiseLevel = 20; _threshAuto = true;
  _refractoryMs = 250; _lastBeatMs = 0; _bpm = 0; _bpmSmooth = 0; _ibiEMA = 0; _beatCb = nullptr;
  _notchEnabled = false; _nb0 = _nb1 = _nb2 = 0; _na1 = _na2 = 0; _nz1 = _nz2 = 0; _notchHz = 0;
  _hist_idx = 0; for (uint8_t i=0;i<HIST_LEN;++i) _hist_env[i]=0; _tpl_ready=false; _sqi=0;
  _tick = 0; _lastReadRawTick = 0; _lastReadSigTick = 0; _tMs = 0; _rawADC_isr = 0; _scaled_isr = 0; _proc_isr = 0; _beatFlag_isr = false;
}

void PulseHeartLab::begin(bool autoStart, bool autoCalibrate, uint16_t sampleRateHz) {
  pinMode(_pin, INPUT);
  _sampleRateHz = (sampleRateHz == 0) ? 100 : sampleRateHz;
  _useTimer = true;
  _baseline = 0;
  _calibrated = false;
  _gainQ10 = 1024;
  _span90 = 0;
  _deadband = 0;
  _nextSampleMs = millis();
  _hp_prev_x = _hp_prev_y = 0;
  _lp_sum = 0; _lp_idx = 0; _lp_prev = 0; _ssf_sum = 0; _ssf_idx = 0; for (uint8_t i = 0; i < SSF_WIN; ++i) _ssf_buf[i] = 0;
  for (uint8_t i = 0; i < LP_WIN; ++i) _lp_buf[i] = 0;
  _proc = 0; _thresh = 50; _peakLevel = 100; _noiseLevel = 20; _threshAuto = true;
  _refractoryMs = 250; _lastBeatMs = 0; _bpm = 0; _bpmSmooth = 0; _ibiEMA = 0; _beatCb = nullptr;
  _notchEnabled = false; _nb0 = _nb1 = _nb2 = 0; _na1 = _na2 = 0; _nz1 = _nz2 = 0; _notchHz = 0;
  _hist_idx = 0; for (uint8_t i=0;i<HIST_LEN;++i) _hist_env[i]=0; _tpl_ready=false; _sqi=0;
  _tick = 0; _lastReadRawTick = 0; _lastReadSigTick = 0; _tMs = 0; _rawADC_isr = 0; _scaled_isr = 0; _proc_isr = 0; _beatFlag_isr = false;

  if (autoCalibrate) {
    // Robust calibration: 256-bin histogram and percentiles (p5, p50, p95)
    const unsigned long durationMs = 1500;
    const unsigned long startMs = millis();
    unsigned long intervalMs = 1000UL / _sampleRateHz;
    if (intervalMs == 0) intervalMs = 1;
    unsigned long nextMs = startMs;
  uint16_t hist[256];
    for (int i = 0; i < 256; ++i) hist[i] = 0;
    unsigned long total = 0;

    while (millis() - startMs < durationMs) {
      if (millis() >= nextMs) {
        int v = analogRead(_pin);
        if (v < 0) v = 0; if (v > 1023) v = 1023;
        uint8_t bin = (uint8_t)(v >> 2); // 0..255
        if (hist[bin] != 0xFFFF) hist[bin]++;
        total++;
        nextMs += intervalMs;
      }
    }

    if (total > 0) {
      unsigned long cumsum = 0;
      int bin5 = 0, bin50 = 0, bin95 = 255;
      unsigned long t5 = (total * 5) / 100;
      unsigned long t50 = (total * 50) / 100;
      unsigned long t95 = (total * 95) / 100;
      for (int i = 0; i < 256; ++i) {
        cumsum += hist[i];
        if (cumsum >= t5 && bin5 == 0) bin5 = i;
        if (cumsum >= t50 && bin50 == 0) bin50 = i;
        if (cumsum >= t95) { bin95 = i; break; }
      }
      // Map bin back to ADC (bin center): bin*4 + 2
      int p5 = (bin5 << 2) + 2;
      int p50 = (bin50 << 2) + 2;
      int p95 = (bin95 << 2) + 2;
  _baseline = p50; // median as robust baseline
      int span = p95 - p5; if (span < 1) span = 1;
  // Scale so that middle 90% ~ +/-512
      long g = ((long)1024 << 10) / (long)span; // Q10
      if (g < 64) g = 64;        // min ~0.0625x
      if (g > 8192) g = 8192;    // max ~8x
      _gainQ10 = (int16_t)g;
  _span90 = span;
  _deadband = span / 20; // ~5% of span as deadband
  if (_deadband < 2) _deadband = 2;
      _calibrated = true;
    }
  }

  if (autoStart) {
    start();
  } else {
    _running = false;
  }
}

void PulseHeartLab::begin(bool autoStart, bool autoCalibrate, uint16_t sampleRateHz, bool useTimer) {
  pinMode(_pin, INPUT);
  _sampleRateHz = (sampleRateHz == 0) ? 100 : sampleRateHz;
  _useTimer = useTimer;
  _baseline = 0;
  _calibrated = false;
  _gainQ10 = 1024;
  _span90 = 0;
  _deadband = 0;
  _nextSampleMs = millis();
  _hp_prev_x = _hp_prev_y = 0;
  _lp_sum = 0; _lp_idx = 0; _lp_prev = 0; _ssf_sum = 0; _ssf_idx = 0; for (uint8_t i = 0; i < SSF_WIN; ++i) _ssf_buf[i] = 0;
  for (uint8_t i = 0; i < LP_WIN; ++i) _lp_buf[i] = 0;
  _proc = 0; _thresh = 50; _peakLevel = 100; _noiseLevel = 20; _threshAuto = true;
  _refractoryMs = 250; _lastBeatMs = 0; _bpm = 0; _bpmSmooth = 0; _ibiEMA = 0; _beatCb = nullptr;
  _notchEnabled = false; _nb0 = _nb1 = _nb2 = 0; _na1 = _na2 = 0; _nz1 = _nz2 = 0; _notchHz = 0;
  _hist_idx = 0; for (uint8_t i=0;i<HIST_LEN;++i) _hist_env[i]=0; _tpl_ready=false; _sqi=0;
  _tick = 0; _lastReadRawTick = 0; _lastReadSigTick = 0; _tMs = 0; _rawADC_isr = 0; _scaled_isr = 0; _proc_isr = 0; _beatFlag_isr = false;

  if (autoCalibrate) {
    const unsigned long durationMs = 1500;
    const unsigned long startMs = millis();
    unsigned long intervalMs = 1000UL / _sampleRateHz; if (intervalMs == 0) intervalMs = 1;
    unsigned long nextMs = startMs;
    uint16_t hist[256]; for (int i = 0; i < 256; ++i) hist[i] = 0; unsigned long total = 0;
    while (millis() - startMs < durationMs) {
      if (millis() >= nextMs) {
        int v = analogRead(_pin);
        if (v < 0) v = 0; if (v > 1023) v = 1023;
        uint8_t bin = (uint8_t)(v >> 2);
        if (hist[bin] != 0xFFFF) hist[bin]++;
        total++; nextMs += intervalMs;
      }
    }
    if (total > 0) {
      unsigned long cumsum = 0; int bin5 = 0, bin50 = 0, bin95 = 255;
      unsigned long t5 = (total * 5) / 100, t50 = (total * 50) / 100, t95 = (total * 95) / 100;
      for (int i = 0; i < 256; ++i) { cumsum += hist[i]; if (cumsum >= t5 && bin5 == 0) bin5 = i; if (cumsum >= t50 && bin50 == 0) bin50 = i; if (cumsum >= t95) { bin95 = i; break; } }
      int p5 = (bin5 << 2) + 2, p50 = (bin50 << 2) + 2, p95 = (bin95 << 2) + 2;
      _baseline = p50; int span = p95 - p5; if (span < 1) span = 1;
      long g = ((long)1024 << 10) / (long)span; if (g < 64) g = 64; if (g > 8192) g = 8192; _gainQ10 = (int16_t)g;
      _span90 = span; _deadband = span / 20; if (_deadband < 2) _deadband = 2; _calibrated = true;
    }
  }

  if (autoStart) start(); else _running = false;
}

void PulseHeartLab::start() {
  _running = true;
  if (_useTimer) { _phl_instance = this; _timerStart(); }
}

void PulseHeartLab::stop() {
  _running = false;
  if (_useTimer) _timerStop();
}

int PulseHeartLab::readRaw() {
  if (!_running) return -1;
  if (_useTimer) {
  // Consume last sample produced by ISR. If none, return -1.
    if (_lastReadRawTick == _tick) return -1;
    _lastReadRawTick = _tick;
    if (_calibrated) return _scaled_isr;
    return _rawADC_isr;
  } else {
    // Polling: sample based on time
    unsigned long now = millis();
    if (now < _nextSampleMs) return -1;
    _nextSampleMs = now + (1000UL / _sampleRateHz);
    int v = analogRead(_pin);
    _rawADC_isr = v;
    int x;
    if (_calibrated) {
      int err = v - _baseline; int abserr = err >= 0 ? err : -err;
      if (abserr <= _deadband) { _baseline += (err >= 0 ? 1 : -1) * ((abserr + 256) >> 9); }
      int centered = v - _baseline; long scaled = ((long)centered * (long)_gainQ10) >> 10;
      if (scaled < -32768) scaled = -32768; if (scaled > 32767) scaled = 32767; x = (int)scaled;
    } else { x = v - 512; }
    _scaled_isr = x;
  // Process to keep pipeline consistent with ISR path
    int env = _applyFilters(x); _proc_isr = env;
    unsigned long tNow = _tMs; _updateDetector(env, tNow); _updateSQIandTemplate();
    _tick++; _tMs += (1000UL / _sampleRateHz);
  _lastReadRawTick = _tick - 1; // allow consumption in this call
    if (_calibrated) return _scaled_isr; else return _rawADC_isr;
  }
}

int PulseHeartLab::getBaseline() {
  return _baseline;
}

int PulseHeartLab::_applyFilters(int x) {
  // Assume x is already centered/scaled if calibrated (done in readRaw)
  // Light HP IIR: y[n] = y[n-1] + x[n] - x[n-1] - alpha*y[n-1] (removes slow drift)
  // Here we use: y = x - prev_x + (hp_prev_y - (hp_prev_y>>5))  (alpha ~ 1/32)
  int dy = x - _hp_prev_x;
  int y = dy + _hp_prev_y - (_hp_prev_y >> 5);
  _hp_prev_x = x;
  _hp_prev_y = y;

  // Optional notch after HP
  if (_notchEnabled) y = _applyNotch(y);

  // Short moving-average LP to smooth
  _lp_sum -= _lp_buf[_lp_idx];
  _lp_buf[_lp_idx] = y;
  _lp_sum += y;
  _lp_idx = (_lp_idx + 1) % LP_WIN;
  int lp = (int)(_lp_sum / LP_WIN);
  // SSF with LP derivative
  int dlp = lp - _lp_prev; _lp_prev = lp;
  int absd = dlp >= 0 ? dlp : -dlp;
  _ssf_sum -= _ssf_buf[_ssf_idx];
  _ssf_buf[_ssf_idx] = absd;
  _ssf_sum += absd;
  _ssf_idx = (_ssf_idx + 1) % SSF_WIN;
  int ssf = (int)_ssf_sum;

  // Simple envelope: absolute value
  int env = lp >= 0 ? lp : -lp;
  int out = env + (ssf >> 2); // add ~25% of SSF
  return out;
}

// Notch biquad DF2T Q15
int PulseHeartLab::_applyNotch(int x) {
  // x is int (assume +-32767). Treat as Q0; coefficients are Q15.
  int32_t in = (int32_t)x; // Q0
  // DF2T: y = b0*x + z1; z1 = b1*x + z2 - a1*y; z2 = b2*x - a2*y
  int32_t y = ((int32_t)_nb0 * in + _nz1) >> 15;
  int32_t z1 = (int32_t)_nb1 * in + _nz2 - ((int32_t)_na1 * y);
  int32_t z2 = (int32_t)_nb2 * in - ((int32_t)_na2 * y);
  _nz1 = z1; _nz2 = z2;
  // y is in Q15, return to Q0
  int32_t yo = y; yo >>= 0; // already scaled by >>15 above
  if (yo < -32768) yo = -32768; if (yo > 32767) yo = 32767;
  return (int)yo;
}

void PulseHeartLab::_updateDetector(int env, unsigned long tNow) {
  // Update adaptive peak/noise levels
  if (env > _thresh) {
    // candidato a pico
    if (env > _peakLevel) _peakLevel = env;
  } else {
  // baseline noise
  if (env > _noiseLevel) _noiseLevel = (_noiseLevel + env) >> 1; // fast average
  }

  // Auto threshold based on peak and noise
  if (_threshAuto) {
    int delta = _peakLevel - _noiseLevel; if (delta < 0) delta = 0;
    _thresh = _noiseLevel + ((delta * 3) >> 2); // ~0.75x acima do noise
    _peakLevel -= (_peakLevel >> 6); // ~1.5% por amostra
    _noiseLevel -= (_noiseLevel >> 7); // decaimento mais lento
    if (_peakLevel < _noiseLevel + 10) _peakLevel = _noiseLevel + 10;
  }

  static bool above = false;
  if (!above && env >= _thresh) {
    above = true;
  }
  // Falling edge crossing indicates pulse apex
  if (above && env < _thresh) {
    above = false;
  // Check refractory
    if (_lastBeatMs == 0 || (tNow - _lastBeatMs) >= _refractoryMs) {
  // Compute BPM with outlier rejection using IBI EMA
      if (_lastBeatMs != 0) {
        unsigned long ibi = tNow - _lastBeatMs; // ms
  if (ibi > 250 && ibi < 2000) { // plausible 30–240 bpm range
          if (_ibiEMA == 0) _ibiEMA = (uint16_t)ibi; else _ibiEMA = (uint16_t)((3UL*_ibiEMA + ibi) / 4UL);
          long diff = (long)ibi - (long)_ibiEMA; long adiff = diff >= 0 ? diff : -diff;
          if (adiff <= (long)_ibiEMA * 3L / 10L) {
            uint16_t bpmInst = (uint16_t)(60000UL / ibi);
            _bpm = bpmInst;
            if (_bpmSmooth == 0) _bpmSmooth = _bpm; else _bpmSmooth = (uint16_t)((7UL*_bpmSmooth + _bpm) / 8UL);
            _beatFlag_isr = true;
          }
        }
      }
      _lastBeatMs = tNow;
    }
  }
}

int PulseHeartLab::readSignal() {
  // Uses same pacing as readRaw; returns -1 if no new sample
  int raw = readRaw();
  if (raw == -1) return -1;
  // Consume last processed sample
  if (_lastReadSigTick == _tick) return -1;
  _lastReadSigTick = _tick;
  int env = _proc_isr;
  // Fire beat callback outside ISR
  if (_beatFlag_isr) { _beatFlag_isr = false; if (_beatCb && _bpmSmooth > 0) _beatCb(_bpmSmooth); }
  return env;
}

uint16_t PulseHeartLab::getBPM() {
  return _bpmSmooth;
}

uint8_t PulseHeartLab::getSQI() { return _sqi; }
// (getThreshold/getPeakLevel/getNoiseLevel) defined below

// Configure notch coefficients for 50/60 Hz (Q15) using current Fs
void PulseHeartLab::setNotch(uint8_t hz) {
  _notchHz = hz; _nz1 = _nz2 = 0; _notchEnabled = false;
  if (!(hz == 50 || hz == 60)) return;
  if (_sampleRateHz < (uint16_t)(hz * 3)) return; // ensure stability margin
  double w0 = 2.0 * 3.14159265358979323846 * (double)hz / (double)_sampleRateHz;
  double c = cos(w0);
  double r = 0.95; // notch width
  // Biquad notch: b0=1, b1=-2cos(w0), b2=1; a1=-2r cos(w0), a2=r^2
  double b0 = 1.0, b1 = -2.0*c, b2 = 1.0;
  double a1 = -2.0*r*c, a2 = r*r;
  auto q15 = [](double v)->int16_t { long t = (long)lrint(v * 32768.0); if (t < -32768) t = -32768; if (t > 32767) t = 32767; return (int16_t)t; };
  _nb0 = q15(b0); _nb1 = q15(b1); _nb2 = q15(b2);
  _na1 = q15(a1); _na2 = q15(a2);
  _notchEnabled = true;
}

void PulseHeartLab::_updateSQIandTemplate() {
  // Update history
  _hist_env[_hist_idx] = (int16_t)_proc_isr; _hist_idx = (_hist_idx + 1) % HIST_LEN;
  // Update a simple template when a beat occurs: moving average of last TPL_LEN points
  if (_beatFlag_isr) {
  // Build template by looking back TPL_LEN samples
    long sum = 0; int16_t tmp[TPL_LEN];
    for (uint8_t i=0;i<TPL_LEN;++i) {
      int idx = (int)_hist_idx - 1 - i; if (idx < 0) idx += HIST_LEN;
      int16_t v = _hist_env[idx]; tmp[TPL_LEN-1-i] = v; sum += v>=0? v: -v;
    }
    long meanAbs = (sum / TPL_LEN); if (meanAbs == 0) meanAbs = 1;
    for (uint8_t i=0;i<TPL_LEN;++i) {
      long s = (long)tmp[i] * 100L / meanAbs; if (s > 127) s = 127; if (s < -127) s = -127;
      _tpl[i] = (int8_t)s;
    }
    _tpl_ready = true;
  }
  // SQI: simple normalized correlation between recent window and template
  if (_tpl_ready) {
    long num=0, den1=0, den2=0;
    for (uint8_t i=0;i<TPL_LEN;++i) {
      int idx = (int)_hist_idx - TPL_LEN + i; if (idx < 0) idx += HIST_LEN;
      int16_t v = _hist_env[idx]; int av = v>=0? v: -v; if (av>32767) av=32767;
      int8_t t = _tpl[i]; int at = t>=0? t: -t;
      num += (long)av * (long)at;
      den1 += (long)av * (long)av;
      den2 += (long)at * (long)at;
    }
    if (den1>0 && den2>0) {
      long sq = (num * 100L) / (long)(sqrt((double)den1 * (double)den2)+1e-6);
      if (sq < 0) sq = 0; if (sq > 100) sq = 100; _sqi = (uint8_t)sq;
    }
  }
}

int PulseHeartLab::getThreshold() { return _thresh; }
int PulseHeartLab::getPeakLevel() { return _peakLevel; }
int PulseHeartLab::getNoiseLevel() { return _noiseLevel; }

void PulseHeartLab::onBeat(void (*cb)(uint16_t bpm)) {
  _beatCb = cb;
}

void PulseHeartLab::setRefractory(uint16_t ms) { _refractoryMs = ms; }
void PulseHeartLab::setThresholdAuto(bool enabled) { _threshAuto = enabled; }

void PulseHeartLab::_timerStart() {
  // Configure Timer2 for CTC at ~100 Hz
  cli();
  TCCR2A = 0; TCCR2B = 0; TCNT2 = 0;
  // Prescaler 128 -> 16MHz/128 = 125kHz; for 100Hz we'd need OCR2A=1249 but 8-bit limits to 255.
  // Use prescaler 1024 -> 15625 Hz; OCR2A = 15625/100 - 1 = 155
  TCCR2A |= (1 << WGM21); // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024
  unsigned long base = (F_CPU / 1024UL);
  unsigned long target = (_sampleRateHz == 0) ? 100 : _sampleRateHz;
  unsigned long ocr = base / target;
  if (ocr > 0) ocr -= 1;
  if (ocr > 255) ocr = 255;
  if (ocr < 1) ocr = 1;
  OCR2A = (uint8_t)ocr;
  TIMSK2 |= (1 << OCIE2A); // enable compare match A interrupt
  sei();
}

void PulseHeartLab::_timerStop() {
  cli();
  TIMSK2 &= ~(1 << OCIE2A);
  TCCR2A = 0; TCCR2B = 0; TCNT2 = 0;
  sei();
}

void PulseHeartLab::onSampleISR() {
  // ADC read
  int v = analogRead(_pin);
  _rawADC_isr = v;

  // Calibration applied: center/scale
  int x = v;
  if (_calibrated) {
    int err = v - _baseline;
    int abserr = err >= 0 ? err : -err;
    if (abserr <= _deadband) {
      _baseline += (err >= 0 ? 1 : -1) * ((abserr + 256) >> 9);
    }
    int centered = v - _baseline;
    long scaled = ((long)centered * (long)_gainQ10) >> 10;
    if (scaled < -32768) scaled = -32768; if (scaled > 32767) scaled = 32767;
    x = (int)scaled;
  } else {
    x = v - 512;
  }
  _scaled_isr = x;

  // Lightweight filters
  int env = _applyFilters(x);
  _proc_isr = env;

  // Detector and BPM
  unsigned long tNow = _tMs;
  _updateDetector(env, tNow);
  _updateSQIandTemplate();

  // Timekeeping
  _tick++;
  _tMs += (1000UL / _sampleRateHz);
}

ISR(TIMER2_COMPA_vect) {
  if (_phl_instance) _phl_instance->onSampleISR();
}
