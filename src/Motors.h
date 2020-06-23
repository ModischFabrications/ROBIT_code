#pragma once

#include <Arduino.h>

class Motors {
  private:
    const uint8_t PINS_LEFT[2] = {10, 9};
    const uint8_t PINS_RIGHT[2] = {6, 5};

    const uint8_t MAX_SPEED = 100; // 0 to 255

    // TODO: pass in from user?
    int8_t motorTuningLeftToRight = -5;

    void setMotorSpeed(const uint8_t* pin_tuple, const bool forwards, const uint8_t speed) const{
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
        int16_t speed = 255 - d * MAX_SPEED + motorTuningLeftToRight;
        setMotorSpeed(PINS_LEFT, d > 0, speed);
    }

    /**
     * set speed of right motor in percentage.
     * -1 is full speed backwards, 1 full speed forwards
     */
    void setRightSpeed(const float factor) {
        float d = constrain(factor, -1, 1);
        int16_t speed = 255 - d * MAX_SPEED - motorTuningLeftToRight;
        setMotorSpeed(PINS_RIGHT, d > 0, speed);
    }
};
