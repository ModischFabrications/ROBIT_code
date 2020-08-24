/**
 * \class Motors
 *
 * \brief Level 1 class for Motors
 *
 * This class allows the control of the two dc motors separately by setting
 * their respective speed as float values between -1.0 and 1.0. Previously the
 * setup function 'begin' should be called to initialise the motor pins.
 * Dedicated getter functions return the respective speed setting.
 */

#pragma once

#include <Arduino.h>

class Motors {
  private:
    const uint8_t PINS_LEFT[2] = {5, 6};
    const uint8_t PINS_RIGHT[2] = {3, 11};

    const uint8_t MIN_SPEED = 80;  // 0 to 255, needs to be lower than MAX_SPEED
    const uint8_t MAX_SPEED = 140; // 0 to 255

    // hardware specific
    int8_t motorRightTuning = -10;

    float leftSpeed = 0;
    float rightSpeed = 0;

    void setMotorSpeed(const uint8_t* pin_tuple, const bool forwards, const uint16_t speed) const {
        if (speed == 0) {
            digitalWrite(pin_tuple[0], LOW);
            digitalWrite(pin_tuple[1], LOW);
        }

        uint8_t active_pin = forwards ? pin_tuple[1] : pin_tuple[0];
        uint8_t disabled_pin = forwards ? pin_tuple[0] : pin_tuple[1];

        analogWrite(active_pin, speed);
        digitalWrite(disabled_pin, LOW);
    }

  public:
    void begin(const int8_t motorRightTuning) {
        pinMode(PINS_LEFT[0], OUTPUT);
        digitalWrite(PINS_LEFT[0], LOW);
        pinMode(PINS_LEFT[1], OUTPUT);
        digitalWrite(PINS_LEFT[1], LOW);
        pinMode(PINS_RIGHT[0], OUTPUT);
        digitalWrite(PINS_RIGHT[0], LOW);
        pinMode(PINS_RIGHT[1], OUTPUT);
        digitalWrite(PINS_RIGHT[1], LOW);

        this->motorRightTuning = motorRightTuning;
    }

    /**
     * set speed of left motor in percentage.
     * -1 is full speed backwards, 1 full speed forwards
     */
    void setLeftSpeed(const float factor) {
        float d = constrain(factor, -1, 1);
        if (this->leftSpeed == d)
            return;
        this->leftSpeed = d;

        uint16_t speed = abs(d) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
        speed = d == 0 ? 0 : speed;

        setMotorSpeed(PINS_LEFT, d > 0, speed);
/*         DEBUG_PRINT("left motor speed: ");
        DEBUG_PRINT(d > 0 ? "+" : "-");
        DEBUG_PRINTLN(speed); */
    }

    float getLeftSpeed() const { return this->leftSpeed; }

    /**
     * set speed of right motor in percentage.
     * -1 is full speed backwards, 1 full speed forwards
     */
    void setRightSpeed(const float factor) {
        float d = constrain(factor, -1, 1);
        if (this->rightSpeed == d)
            return;
        this->rightSpeed = d;

        uint16_t speed = abs(d) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED + motorRightTuning;
        speed = d == 0 ? 0 : speed;

        setMotorSpeed(PINS_RIGHT, d > 0, speed);
/*         DEBUG_PRINT("right motor speed: ");
        DEBUG_PRINT(d > 0 ? "+" : "-");
        DEBUG_PRINTLN(speed); */
    }

    float getRightSpeed() const { return this->rightSpeed; }
};
