#pragma once

#include <Arduino.h>

class Motors {
  private:
    const uint8_t PINS_LEFT[2] = {5, 6};
    const uint8_t PINS_RIGHT[2] = {9, 10};

    const uint8_t MIN_SPEED = 70; // 0 to 255
    const uint8_t MAX_SPEED = 110; // 0 to 255

    // TODO: pass in from user?
    int8_t motorRightTuning = -18;

    void setMotorSpeed(const uint8_t* pin_tuple, const bool forwards, const uint16_t speed) const{
        if (speed == 0) {
            digitalWrite(pin_tuple[0], LOW);
            digitalWrite(pin_tuple[1], LOW);
        }

        uint8_t active_pin = forwards ? pin_tuple[1] : pin_tuple[0];
        uint8_t disabled_pin = forwards ? pin_tuple[0] : pin_tuple[1];

        analogWrite(active_pin, speed);
        digitalWrite(disabled_pin, LOW);
    }

    // TODO: cache speeds?

  public:
    Motors() {
        pinMode(PINS_LEFT[0], OUTPUT);
        digitalWrite(PINS_LEFT[0], LOW);
        pinMode(PINS_LEFT[1], OUTPUT);
        digitalWrite(PINS_LEFT[1], LOW);
        pinMode(PINS_RIGHT[0], OUTPUT);
        digitalWrite(PINS_RIGHT[0], LOW);
        pinMode(PINS_RIGHT[1], OUTPUT);
        digitalWrite(PINS_RIGHT[1], LOW);
    }

    /**
     * set speed of left motor in percentage.
     * -1 is full speed backwards, 1 full speed forwards
     */
    void setLeftSpeed(const float factor) {
        float d = constrain(factor, -1, 1);
        uint16_t speed = abs(d) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
        speed = d == 0 ? 0 : speed;

        setMotorSpeed(PINS_LEFT, d > 0, speed);
    }

    /**
     * set speed of right motor in percentage.
     * -1 is full speed backwards, 1 full speed forwards
     */
    void setRightSpeed(const float factor) {
        float d = constrain(factor, -1, 1);
        uint16_t speed = abs(d) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED + motorRightTuning;
        speed = d == 0 ? 0 : speed;

        setMotorSpeed(PINS_RIGHT, d > 0, speed);
    }
};
