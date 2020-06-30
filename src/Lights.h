#pragma once

#include <Arduino.h>
#define FASTLED_INTERNAL // disable pragma message
#include <FastLED.h>

class Lights {
  private:
    // statics needed to allow FastLED template parameters
    // TODO: use correct pin
    static const uint8_t PIN_LEDS = 8;
    static const uint8_t N_LEDS = 10;

    const uint16_t MAX_MILLIAMPS = 300;

    const uint16_t T_ANIMATION_MS = (1 * 1000);

    CRGB leds[N_LEDS];

  public:
    Lights() {
        pinMode(PIN_LEDS, OUTPUT);

        FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, N_LEDS);
        // set powerlimit to 5v, 1000mA (Fuse size)
        FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_MILLIAMPS);

        FastLED.showColor(CRGB::Black);
        // TODO: might be useless, evaluate
        FastLED.show();
    }

    // flash leds one by one
    void helloPower() {
        // TODO: use fading, might want to reuse clock functions
        // might even want to use a rainbow fade? -> check FastLED buildin animations
        for (uint8_t i = 0; i < N_LEDS; i++) {
            leds[i] = CRGB::White;
            FastLED.show();
            delay(T_ANIMATION_MS / N_LEDS);
            leds[i] = CRGB::Black;
        }
    }

    /**
     * Use this delay to ensure correct display of lights
     * */
    void delay(uint32_t time_ms) { FastLED.delay(time_ms); }

    // TODO: allow access to fastled methods? indirection here gives no value ...
};
