#pragma once

#include <Arduino.h>

class LineSensor {
  private:
    const uint8_t PIN_LINESENSOR = 2; // only 2 & 3 work
    bool LIGHTFLOOR = false; // set the color of the floor, adjust the color of the tape accordingly

  public:
    void begin(const bool lightFloor) {
      LIGHTFLOOR = lightFloor;
      pinMode(PIN_LINESENSOR, INPUT);
    }

    void registerListener(void (*listener)(void)) {
        attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), listener, LIGHTFLOOR ? FALLING : RISING);
    }

    /**
     * Check if line is currently detected
     * */
    bool detected() {
        bool detected = LIGHTFLOOR ? digitalRead(PIN_LINESENSOR) : !digitalRead(PIN_LINESENSOR);
        DEBUG_PRINT("line detected? ");
        DEBUG_PRINTLN(detected);
        return detected;
    }
};
