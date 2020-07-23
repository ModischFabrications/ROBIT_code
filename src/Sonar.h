#pragma once

#include "Ultrasonic.h"
#include <Arduino.h>

class Sonar {
    // level 2 convenience class for Ultrasonic
  private:
    Ultrasonic ultrasonic = Ultrasonic();

  public:
    // too many measurements will increase time cost dramatically
    const uint8_t n_measurements = 10;
    static constexpr float SOUND_SPEED = 29.15f;
    static const uint8_t MAX_DISTANCE = 100;
    // static const uint32_t MAX_TIME = (2L * MAX_DISTANCE) * SOUND_SPEED + 10;

    void begin() { ultrasonic.begin(); }

    uint8_t get_raw_distance() const {
        uint16_t time_us = ultrasonic.poll_time_us();
        uint8_t distance_cm = (time_us / 2) / SOUND_SPEED;

        distance_cm = constrain(distance_cm, 0, MAX_DISTANCE);

        return distance_cm;
    }

    uint8_t get_min_distance() const {
        uint8_t min_distance = MAX_DISTANCE;

        for (uint8_t i = 0; i < n_measurements; i++) {
            uint8_t this_dist = get_raw_distance();
            if (this_dist < min_distance)
                min_distance = this_dist;
            // make sure no old signals are being detected and get some deviation
            delay(1);
        }

/*         DEBUG_PRINT("Sonar distance: ");
        DEBUG_PRINTLN(min_distance); */
        return min_distance;
    }
};
