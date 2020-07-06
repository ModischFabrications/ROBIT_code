#pragma once

#include <Arduino.h>

class LineSensor {
  private:
    const uint8_t PIN_LINESENSOR = 2;   // only 2 & 3 work

  public:
    void begin() { pinMode(PIN_LINESENSOR, INPUT); }

    void registerListener(void (*listener)(void)) {
        attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), listener, RISING);
    }

    /**
     * Check if line is currently detected
     * */
    bool detected() {
        return digitalRead(PIN_LINESENSOR);
    }
};