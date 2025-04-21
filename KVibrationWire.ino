/* STEP 2 — Branch code (rev B2 — set as new constant)
   ─────────────────────────────────────────
   • Keeps hardware setup from STEP 1  
   • Corrected FFT peak calculation for arduinoFFT v2.x  
   • Ready for calibration (1 kHz in → reports ~1 kHz)
*/

#include <driver/adc.h>
#include <driver/ledc.h>
#include "arduinoFFT.h"          // arduinoFFT v2.x

const uint8_t ADC_PIN     = 34;   // INA OUT → ADC1_CH6
const uint8_t NTC_PIN     = 35;   // NTC divider
const uint8_t MOSFET_PIN  = 25;   // pluck gate (keep but not used during ext‑gen test)

const uint16_t SAMPLES     = 512;     // power‑of‑2 for FFT
const double SAMPLE_RATE = 2086.0;  // ADC sampling rate in Hz

static double vReal[SAMPLES];
static double vImag[SAMPLES];

ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLE_RATE);

void setup() {
  Serial.begin(115200);
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten((adc1_channel_t)ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten((adc1_channel_t)ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  Serial.println(F("VW Gauge Reader — STEP 2 rev B2 (FFT v2.x)"));
}

inline void pluck(uint16_t ms = 5) {
  // keep the function for real sensor; ignored during ext‑gen calibration
  digitalWrite(MOSFET_PIN, HIGH);
  delay(ms);
  digitalWrite(MOSFET_PIN, LOW);
}

void capture() {
  const uint32_t us = 1000000UL / (uint32_t)SAMPLE_RATE;
  for (uint16_t i = 0; i < SAMPLES; ++i) {
    vReal[i] = adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_6);
    vImag[i] = 0.0;
    delayMicroseconds(us);
  }
}

void loop() {
  static uint32_t t0 = 0;
  if (millis() - t0 >= 1000) {   // між імпульсами
    pluck();        // ← розкоментуйте після підключення датчика
    capture();

    uint16_t peak = 0;
    for (uint16_t i = 0; i < SAMPLES; ++i) if (vReal[i] > peak) peak = (uint16_t)vReal[i];
    Serial.printf("Peak:%u\t", peak);

    // FFT analysis — in v2.x majorPeak() returns the frequency directly
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    double freq = FFT.majorPeak();
    Serial.printf("Freq:%.1f Hz\t", freq);

    uint16_t ntcRaw = adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_7);
    Serial.printf("NTC:%u\n", ntcRaw);

    t0 = millis();
    delay(1000);
  }
}


