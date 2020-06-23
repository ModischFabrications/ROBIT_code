#pragma once

#include <Arduino.h>

class Ultrasonic {
  private:
    const uint8_t PIN_TRG = 8;
    const uint8_t PIN_ECHO = 9;

  public:
    const uint16_t MAX_TIME = 26240;
    const uint16_t MAX_DIST = 500;

    Ultrasonic() {
        pinMode(PIN_TRG, OUTPUT);
        digitalWrite(PIN_TRG, LOW);

        pinMode(PIN_ECHO, INPUT);
    }

    /**
     * returns time for reflection in microseconds
     */
    uint16_t poll_time_us() {
        // trigger measurement
        digitalWrite(PIN_TRG, HIGH);
        delayMicroseconds(20); // experiment a bit, at least 10 us
        digitalWrite(PIN_TRG, LOW);

        // travel time in us, limit to max range (<200cm reasonable)
        uint32_t duration = pulseIn(PIN_ECHO, HIGH, MAX_TIME); 

        // TODO: wait remaining time for constant execution time

        return duration;
    }

    // TODO: --------- extract to user code?

    uint8_t get_distance() {
        uint16_t time_us = poll_time_us();
        uint8_t distance_cm = (time_us / 2) / 29.1545;

        return distance_cm;
    }

    uint8_t get_min_distance() {
        uint8_t min_distance = MAX_DIST;

        for (uint8_t i = 0; i < 10; i++) {
            uint8_t this_dist = get_distance();
            if (this_dist < min_distance)
                min_distance = this_dist;
            delay(1);
        }

        return min_distance;
    }
};
