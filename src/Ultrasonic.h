/**
 * \class Ultrasonic
 *
 * \brief Level 1 class for Ultrasonic sensor
 *
 * This class can trigger a ultrasonic ping and measure it's round trip time.
 * This can be used to measure distances of obstacles. Be aware that ultrasonic
 * sensors need a good surface for reflections, look into "radar cross sections"
 * for more hints about their design. Call 'begin' prior to usage to initialise
 * the sensor pins.
 */

#pragma once

#include <Arduino.h>

class Ultrasonic {
  private:
    const uint8_t PIN_TRG = 8;
    const uint8_t PIN_ECHO = 7;

  public:
    // this is the max limit to get a reflection from, better sensors can be used much further.
    // 5800 ~= 1m
    static const uint32_t MAX_TIME = 5800;

    void begin() {
        pinMode(PIN_TRG, OUTPUT);
        digitalWrite(PIN_TRG, LOW);

        // HC-SR04 should have an internal pulldown
        pinMode(PIN_ECHO, INPUT);
    }

    /**
     * returns time from trigger to detection of reflection in microseconds.
     * MAX_TIME means nothing was found.
     */
    uint16_t poll_time_us() const {
        // trigger measurement
        digitalWrite(PIN_TRG, HIGH);
        delayMicroseconds(10); // experiment a bit, at least 10 us
        digitalWrite(PIN_TRG, LOW);

        // longer echo time means an object is further away than allowed or nothing was detected.
        uint32_t duration = pulseIn(PIN_ECHO, HIGH, MAX_TIME);

        duration = (duration > 0) ? duration : MAX_TIME;

        // wait remaining time for constant execution time
        delayMicroseconds(MAX_TIME - duration);

        return duration;
    }
};
