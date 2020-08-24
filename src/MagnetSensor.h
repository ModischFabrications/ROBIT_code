/**
 * \class MagnetSensor
 *
 * \brief Level 1 class for Magnet sensor
 *
 * This class allows reading the state of the Magnet sensor, therefore the
 * function 'detected' is used. Previously the setup function 'begin' should be
 * called to initialise the sensor pin.
 */

#pragma once

#include <Arduino.h>

class MagnetSensor {
  private:
    const uint8_t PIN_HALL = A3;

  public:
    void begin() { pinMode(PIN_HALL, INPUT); }

    /**
     * True if a magnet is close.
     * */
    bool detected() {
        bool detected = digitalRead(PIN_HALL);
        DEBUG_PRINT("magnet detected? ");
        DEBUG_PRINTLN(detected);
        return detected;
    }
};
