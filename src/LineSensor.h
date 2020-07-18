#pragma once

#include <Arduino.h>

class LineSensor {
  private:
    const uint8_t PIN_LINESENSOR = 2; // only 2 & 3 work

  public:
    void begin() { pinMode(PIN_LINESENSOR, INPUT); }

    void registerListener(void (*listener)(void)) {
        attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), listener, FALLING);
    }

    /**
     * Check if line is currently detected
     * */
    bool detected() {
        bool detected = digitalRead(PIN_LINESENSOR);
        DEBUG_PRINT("line detected? ");
        DEBUG_PRINTLN(detected);
        return detected;
    }
};
