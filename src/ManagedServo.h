/**
 * \class ManagedServo
 *
 * \brief Level 2 class for Servo
 *
 * This class is based on Servo.h, it extends the class by five functions which
 * make it possible to move the servo within the physical possible positions in
 * a controlled speed. The function 'begin' should be called to initialise the
 * servo pins and to set the limit positions.
 * The function 'update' has to be called in every update loop.
 * To move the servo to its limit positions the function 'moveDown' and 'moveUp'
 * is used.
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

  void moveDown() {
    servo.goToMin();
  }

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
