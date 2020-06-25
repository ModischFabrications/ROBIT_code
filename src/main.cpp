#include <Arduino.h>
#include <TinyMPU6050.h>

#include "Motors.h"
#include "Ultrasonic.h"

MPU6050 mpu(Wire);

Ultrasonic ultrasonic;
Motors motor;

const uint8_t PIN_LINESENSOR = 2;   // only 2 & 3 work

int16_t angleZ(); // 0 to 359

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

const uint16_t reverseForMS = 2000;
volatile uint32_t reverseUntilTime = 0;

const uint8_t pickupDistance = 5;

void startSearch() {
    initial_angle = angleZ();
    smallestDistanceFound = ultrasonic.MAX_DISTANCE;
    angleOfSmallestDistance = initial_angle;
    motor.setLeftSpeed(-0.1);
    motor.setRightSpeed(0.1);

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
    motor.setLeftSpeed(1);
    motor.setRightSpeed(1);

    state = approachState;
}

void startPickup() {
    motor.setLeftSpeed(0);
    motor.setRightSpeed(0);

    state = pickupState;
}

void line_found() {
    if (state == returnState) {
        state = finalState;
        // done!
        return;
    }

    // assume we only hit the line on forward movements
    startReverse();
}

void setup() {
    Serial.begin(115200);
    mpu.Initialize();
    // mpu.Calibrate();

    attachInterrupt(digitalPinToInterrupt(PIN_LINESENSOR), line_found, RISING);
}

void loop() {
    mpu.Execute();

    switch (state) {
    case initState: {
        startSearch();
    } break;

    case searchState: {
        uint16_t current_distance = ultrasonic.get_distance();
        int16_t current_angle = angleZ();
        if (current_distance < smallestDistanceFound) {
            // found something closer
            smallestDistanceFound = current_distance;
            angleOfSmallestDistance = current_angle;
        }

        if (angleZ() - initial_angle >= 360) {
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
        if (angleZ() == angleOfSmallestDistance) {
            startApproach();
        }

    } break;

    case approachState: {
        uint16_t distance = ultrasonic.get_distance();

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
        // TODO: try to pick it up multiple times, reposition if unsuccessful
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
    delay(targetLoopDuration - (lastLoopTime % targetLoopDuration));
}

int16_t angleZ() { return mpu.GetAngZ() + 179; }
