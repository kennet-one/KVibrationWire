#include <Wire.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <U8g2lib.h>
#include "arduinoFFT.h"

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(
  U8G2_R0,
  /* cs=*/ 5,
  /* dc=*/ 16,
  /* reset=*/ 17
);

const uint8_t ADC_PIN     = 34;  // INA OUT → ADC1_CH6
const uint8_t NTC_PIN     = 35;  // NTC divider
const uint8_t MOSFET_PIN  = 25;  // pluck gate

const uint16_t SAMPLES     = 512;
const double   SAMPLE_RATE = 2086.0;

static double vReal[SAMPLES], vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLE_RATE);

void setup() {
  Serial.begin(115200);
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  u8g2.begin();
  u8g2.setContrast(0);
}

inline void pluck(uint16_t ms = 5) {
  digitalWrite(MOSFET_PIN, HIGH);
  delay(ms);
  digitalWrite(MOSFET_PIN, LOW);
}

void capture() {
  const uint32_t us = 1000000UL / (uint32_t)SAMPLE_RATE;
  for (uint16_t i = 0; i < SAMPLES; ++i) {
    vReal[i] = adc1_get_raw(ADC1_CHANNEL_6);
    vImag[i] = 0.0;
    delayMicroseconds(us);
  }
}

void loop() {
  static uint32_t t0 = 0;
  if (millis() - t0 >= 1000) {   // між імпульсами
    t0 = millis();

    // 1) Пульс для датчика (закоментуйте під час тесту з генератором)
    pluck();

    // 2) Захоплення даних
    capture();

    // 3) Підрахунок піку
    uint16_t peak = 0;
    for (uint16_t i = 0; i < SAMPLES; ++i)
      if (vReal[i] > peak) peak = (uint16_t)vReal[i];

    // 4) FFT і частота
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    double freq = FFT.majorPeak();

    // 5) Зчитування NTC
    uint16_t ntcRaw = adc1_get_raw(ADC1_CHANNEL_7);

    // 6) Серіал (для логів)
    Serial.printf("Peak:%u\tFreq:%.1f Hz\tNTC:%u\n", peak, freq, ntcRaw);

    // 7) Оновлення дисплея
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tf);

      u8g2.setCursor(0, 12);
      u8g2.print("Peak: ");
      u8g2.print(peak);

      u8g2.setCursor(0, 26);
      u8g2.print("Freq: ");
      u8g2.print(freq, 1);
      u8g2.print(" Hz");

      u8g2.setCursor(0, 40);
      u8g2.print("NTC: ");
      u8g2.print(ntcRaw);

      u8g2.setCursor(0, 54);
      u8g2.print("t=");
      u8g2.print(t0 / 1000);
      u8g2.print("s");
    } while (u8g2.nextPage());
  }
}
