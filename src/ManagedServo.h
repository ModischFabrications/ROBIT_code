/**
 * \class ManagedServo
 *
 * \brief Level 2 class for Servo
 *
 * This class is based on Servo.h.It extends Servo.h by constraining the
 * possible range and giving a finer control over movement speeds. Call 'begin'
 * prior to usage to initialise the servo pins and to set the limit positions.
 * The function 'update' has to be called in every update loop to interpolate
 * between the set positions.
 */

#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <SlowMotionServo.h>

class ManagedServo {
// level 2 convenience class for Servo
private:
  const uint8_t PIN_SERVO = 13;
  SMSSmooth servo;


public:
  void begin() {
    // endstops tuned for specific servo
    servo.setMin(900);
    servo.setMax(2100);
    servo.setSpeed(10);
    servo.setInitialPosition(0.9);
    servo.setDetachAtMax(true);

    // servo needs 0.5 sec for full movement
    servo.setDelayUntilStop(500);

    servo.setPin(PIN_SERVO);
  }

  void update() {
    SlowMotionServo::update();
  }

  /**
   * move to lower limit position
   * */
  void moveDown() {
    servo.goToMin();
  }

  /**
   * move to upper limit position
   * */
  void moveUp() {
    servo.goToMax();
  }

  bool isStopped() {
    return servo.isStopped();
  }

  void waitUntilStopped() {
    while (!servo.isStopped())
    {
      this->update();
      delay(1);
    }
  }
};
