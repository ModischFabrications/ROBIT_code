#pragma once

#include<Arduino.h>


class Ultrasonic {
    private:
        const uint8_t PIN_TRG = 8;
        const uint8_t PIN_ECHO = 9;

    public:
        const uint16_t MAX_TIME = 26240;

        Ultrasonic() {
            pinMode(PIN_TRG, OUTPUT);
            digitalWrite(PIN_ECHO, LOW);
        
            pinMode(PIN_ECHO, INPUT);
        }

        /**
         * returns time for reflection in microseconds
         */
        uint16_t poll_time_us() {
        // trigger measurement
        digitalWrite(PIN_TRG, HIGH);
        delayMicroseconds(100);  //experiment a bit, at least 10 us
        digitalWrite(PIN_TRG, LOW);
        
        uint32_t duration = pulseIn(PIN_ECHO, HIGH, MAX_TIME); //travel time in us, limit to max range (<200cm reasonable)

        // TODO: wait remaining time for constant execution time

        return duration;
        }

        // TODO: extract to user code?
        uint8_t get_distance() {
            uint16_t time_us = poll_time_us();
            uint8_t distance_cm = (time_us / 2) / 29.1545;

            return distance_cm;
        }

};

