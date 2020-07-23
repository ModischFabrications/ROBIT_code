#pragma once

#include <Arduino.h>

#include "Gyro.h"
#include "Motors.h"

class ManagedMotors {
    // level 2 convenience class for Motors
  private:
    // experimental results
    const int8_t MOTOR_RIGHT_TUNING = -15;

    const uint8_t P_REGULATION_FACTOR = 40; // smaller is stronger compensation
    const float MAX_TRIM_FACTOR = 0.5f;

    Motors motors = Motors();
    Gyro gyro;

    bool trim = false;
    int16_t targetAngle;

    float targetSpeeds[2]; // left, right

    void setSpeeds(float left, float right) {
        targetSpeeds[0] = left;
        targetSpeeds[1] = right;

        motors.setLeftSpeed(left);
        motors.setRightSpeed(right);
    }

    void trim_direction(int16_t curr_angle, int16_t target_angle) {
        // changing speed of side that drifts off or lags behind might collide with min_speed
        // -> both need to be changed
        float trim_factor = (curr_angle - target_angle) / float(P_REGULATION_FACTOR);

        trim_factor = constrain(trim_factor, -MAX_TRIM_FACTOR, MAX_TRIM_FACTOR);

        motors.setLeftSpeed(targetSpeeds[0] - trim_factor);
        motors.setRightSpeed(targetSpeeds[1] + trim_factor);
    }

  public:
    ManagedMotors(const Gyro gyro) { this->gyro = gyro; }

    void begin() { motors.begin(MOTOR_RIGHT_TUNING); }

    /**
     * positive turns right. This won't always turn with equal speed!
     * */
    void turn(float speed) {
        speed = constrain(speed, -1, 1);
        setSpeeds(speed, -speed);
        trim = false;
    }

    /**
     * positive is forwards, reverse is possible.
     * This is a controlled movement, deviations will be corrected internally.
     * */
    void move(float speed) {
        if (speed == 0) {
            stop();
            return;
        }

        speed = constrain(speed, -1, 1);
        setSpeeds(speed, speed);
        targetAngle = gyro.getAngleZ();
        trim = true;
    }

    /**
     * same as speed == 0
     * */
    void stop() {
        setSpeeds(0, 0);
        trim = false;
    }

    /**
     * necessary to trim movement direction from gyro readouts
     * */
    void update() {
      if (trim) {
        gyro.update();  // better safe than sorry
        int16_t curr_angle = gyro.getAngleZ();
        if (curr_angle != targetAngle) {
            trim_direction(curr_angle, targetAngle);
        }
      }
    }
};
