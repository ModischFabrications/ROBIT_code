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
        return digitalRead(PIN_HALL);
    }
};
