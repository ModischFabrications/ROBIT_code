/**
 * \class LineSensor
 *
 * \brief Level 1 class for Line sensor
 *
 * This class allows reading the state of the Line sensor, therefore the
 * function 'detected' is used. It also supports the registration of a listener
 * based on a interrupt pin. To configure a function which will be called the
 * function 'registerListener' is used. 'removeListener' detaches the registered
 * listener. Previously the setup function 'begin' should be called to
 * initialise the sensor pin and to set the floor color type.
 */


#pragma once

#include <Arduino.h>

class LineSensor {
  private:
    const uint8_t PIN_LINESENSOR = 2; // only 2 & 3 work
    bool light_floor = false; // set the color of the floor, adjust the color of the tape accordingly

  public:
    /**
     * true: using a floor with a light color, false: floor with a dark color
     * */
    void begin(const bool lightFloor) {
      light_floor = lightFloor;
      pinMode(PIN_LINESENSOR, INPUT);
    }

    /**
     * monitor line sensor to detect out of bounds
     * */
    void registerListener(void (*listener)(void)) {
        attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), listener, light_floor ? FALLING : RISING);
    }

    /**
     * stop monitoring, eg. if everything is done.
     * */
    void removeListener() {
      detachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR));
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
