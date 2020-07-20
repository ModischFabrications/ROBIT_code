#pragma once

#include <Arduino.h>

class LineSensor {
  private:
    const uint8_t PIN_LINESENSOR = 2; // only 2 & 3 work
    bool light_floor = false; // set the color of the floor, adjust the color of the tape accordingly

  public:
    void begin(const bool lightFloor) {
      light_floor = lightFloor;
      pinMode(PIN_LINESENSOR, INPUT);
    }

    void registerListener(void (*listener)(void)) {
        attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), listener, light_floor ? FALLING : RISING);
    }

    /**
     * Check if line is currently detected
     * */
    bool detected() {
        bool detected = light_floor ? digitalRead(PIN_LINESENSOR) : !digitalRead(PIN_LINESENSOR);
        DEBUG_PRINT("line detected? ");
        DEBUG_PRINTLN(detected);
        return detected;
    }
};
