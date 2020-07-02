#pragma once

#include <Arduino.h>

class MagnetSensor {
  private:
    // TODO: correct pin
    const uint8_t PIN_HALL = 99;

  public:
    void begin() { pinMode(PIN_HALL, INPUT); }

    /**
     * True if a magnet is close.
     * */
    bool detected() {
        // TODO: invert signal if needed
        return digitalRead(PIN_HALL);
    }
};
