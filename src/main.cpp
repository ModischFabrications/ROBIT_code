#include <Arduino.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include "Motors.h"
#include "Ultrasonic.h"
#include "Lights.h"
#include "Gyro.h"

Ultrasonic ultrasonic;
Motors motor;
Lights lights;
Gyro gyro;

const uint8_t PIN_LINESENSOR = 2;   // only 2 & 3 work

enum FSMstates {
    initState,
    searchState,
    alignToTargetState,
    approachState,
    reverseState,
    pickupState,
    returnState,
    finalState
};

volatile FSMstates state = initState;

const uint16_t targetLoopDuration = 20;
uint32_t lastLoopTime = 0;

uint16_t smallestDistanceFound = 300;
int16_t angleOfSmallestDistance = 0;
int16_t initial_angle = 0;

const uint16_t reverseForMS = 1000;
volatile uint32_t reverseUntilTime = 0;

const uint8_t pickupDistance = 5;

const uint8_t p_regulation_factor = 50;

void startSearch() {
    initial_angle = gyro.getAngleZ();
    smallestDistanceFound = ultrasonic.MAX_DISTANCE;
    angleOfSmallestDistance = initial_angle;
    motor.setLeftSpeed(0.5);
    motor.setRightSpeed(-0.5);

    state = searchState;
}

void startReverse() {
    motor.setLeftSpeed(-0.5);
    motor.setRightSpeed(-0.5);
    reverseUntilTime = millis() + reverseForMS;

    state = reverseState;
}

void startAlign() {
    // TODO: find shortest turn direction
    motor.setLeftSpeed(-0.1);
    motor.setRightSpeed(0.1);

    state = alignToTargetState;
}

void startApproach() {
    motor.setLeftSpeed(0.1);
    motor.setRightSpeed(0.1);

    state = approachState;
}

void startPickup() {
    motor.setLeftSpeed(0);
    motor.setRightSpeed(0);

    state = pickupState;
}

void line_found() {
    Serial.println("line_found");
    if (state == returnState) {
        state = finalState;
        // done!
        return;
    }

    // assume we only hit the line on forward movements
    startReverse();
}

void driveTest() {
  while(1) {
    float i = -1;
    for (; i < 1; i += 0.1) {
      motor.setRightSpeed(i);
      motor.setLeftSpeed(i);
      delay(500);
    }
  }
}

void correct_direction(int16_t curr_angle, int16_t target_angle) {
    // changing speed of side that drifts off or lags behind might collide with min_speed
    // -> both need to be changed
    float correction_factor = (curr_angle - target_angle)/float(p_regulation_factor);

    correction_factor = constrain(correction_factor, -0.5f, 0.5f);

    motor.setLeftSpeed(-0.5);
    motor.setRightSpeed(-0.5);

    motor.setLeftSpeed(motor.getLeftSpeed()  - correction_factor);
    motor.setRightSpeed(motor.getRightSpeed() + correction_factor);
}

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    gyro.begin();

    attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), line_found, RISING);
    //driveTest();

    lights.helloPower();
}

void loop() {
    mpu6050.update();

    DEBUG_PRINT("state: ");
    DEBUG_PRINTLN(state);

    switch (state) {
    case initState: {
        startSearch();
    } break;

    case searchState: {
        uint16_t current_distance = ultrasonic.get_distance();
        int16_t current_angle = gyro.getAngleZ();
        if (current_distance < smallestDistanceFound) {
            // found something closer
            smallestDistanceFound = current_distance;
            angleOfSmallestDistance = current_angle;
        }
        if (gyro.getAngleZ() - initial_angle >= 360) {
            // full rotation
            if (smallestDistanceFound == ultrasonic.MAX_DISTANCE) {
                // nothing found
                // TODO: random move to new search position
                return;
            }
            startAlign();
        }


    } break;

    case alignToTargetState: {
        // turn to face shortest distance
        // TODO: approximated comparison, won't hit exact angle
        if (gyro.getAngleZ() == angleOfSmallestDistance) {
            startApproach();
        }

    } break;

    case approachState: {
        int16_t curr_angle = gyro.getAngleZ();
        if (curr_angle != angleOfSmallestDistance) {
            correct_direction(curr_angle, angleOfSmallestDistance);
        }

        uint16_t distance = ultrasonic.get_min_distance();

        if (distance == ultrasonic.MAX_DISTANCE) {
            // lost it again
            startSearch();
            return;
        }

        if (distance <= pickupDistance) {
            // reached it
            startPickup();
            return;
        }

    } break;

    case reverseState: {
        if (millis() > reverseUntilTime)
            // revert everything and try again, we messed up somewhere before
            startSearch();
    } break;

    case pickupState: {
        /* TODO implement async
        assert hall_sensor == 0
        servo down
        if hall_sensor == 1:
            servo up
            returnState
        else:
            reposition and try again
        */

    } break;

    case returnState: {
        // wait for interrupt
        delay(10);
    } break;

    case finalState: {
        // done.
        // TODO: notify user
        delay(1000);
    } break;
    }

    // keep constant loop duration
    lights.delay(targetLoopDuration - (lastLoopTime % targetLoopDuration));
}
