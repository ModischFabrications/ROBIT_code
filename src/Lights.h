/**
 * \class Lights
 *
 * \brief Level 2 class for LEDs
 *
 * This class is based on FastLED.h and it allows the control of the LEDs and
 * setting effects. The function 'helloPower' is startup effect which runs a
 * single pixel across the LED strip. Previously the setup
 * function 'begin' should be called to initialise the LED pins, set the number
 * of LEDS and their power limit.
 */

#pragma once

#include <Arduino.h>
#define FASTLED_INTERNAL // disable pragma message
#include <FastLED.h>

class Lights {
  private:
    // statics needed to allow FastLED template parameters
    static const uint8_t PIN_LEDS = 4;

    const uint16_t MAX_BRIGHTNESS = 100;
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

        // .show() is included
        FastLED.showColor(CRGB::Black);
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
