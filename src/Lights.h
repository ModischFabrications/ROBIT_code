#pragma once

#include <Arduino.h>
#define FASTLED_INTERNAL // disable pragma message
#include <FastLED.h>

class Lights {
  private:
    // statics needed to allow FastLED template parameters
    static const uint8_t PIN_LEDS = 4;

    const uint16_t MAX_BRIGHTNESS = 200;
    const uint16_t MAX_MILLIAMPS = 300;

    const uint16_t T_ANIMATION_MS = (1 * 1000);

  public:
    static const uint8_t N_LEDS = 10;
    CRGB leds[N_LEDS];

    void begin() {
        pinMode(PIN_LEDS, OUTPUT);

        FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, N_LEDS).setCorrection(TypicalLEDStrip);

        // prevent blindness
        FastLED.setBrightness(MAX_BRIGHTNESS);
        // prevent brownouts
        FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_MILLIAMPS);

        FastLED.showColor(CRGB::Black);
        // TODO: might be useless, evaluate
        FastLED.show();
    }

    /**
     * Test LEDs, timing and power supply
     * */
    void helloPower() {
        // TODO: use fading? might even want to use a rainbow fade?
        // -> check FastLED buildin animations
        for (uint8_t i = 0; i < N_LEDS; i++) {
            leds[i] = CRGB::Red;
            FastLED.show();
            FastLED.delay(T_ANIMATION_MS / N_LEDS);
            leds[i] = CRGB::Black;
        }
    }

    /**
     * Use this delay to ensure correct display of lights
     * */
    void delay(uint32_t time_ms) { FastLED.delay(time_ms); }

    // TODO: allow access to fastled methods? indirection here gives no value ...
};
