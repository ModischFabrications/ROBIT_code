#pragma once

#include <Arduino.h>

class LineSensor {
  private:
    // TODO: correct pin
    const uint8_t PIN_LINESENSOR = 2;   // only 2 & 3 work

  public:
    void begin() { pinMode(PIN_LINESENSOR, INPUT); }

    void registerListener(void (*listener)(void)) {
        // TODO: invert flank if needed
        attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), listener, RISING);
    }

    /**
     * Check if line is currently detected
     * */
    bool detected() {
        // TODO: invert signal if needed
        return digitalRead(PIN_LINESENSOR);
    }
};