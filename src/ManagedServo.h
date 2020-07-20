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
    servo.setMax(2200);
    servo.setSpeed(5);
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
