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
    approachState,
    reverseState,
    pickupState,
    returnState,
    finalState
};

volatile FSMstates state = initState;

const uint16_t targetLoopDuration = 10;
uint32_t lastLoopTime = 0;

uint16_t smallestDistanceFound = 300;
int16_t angleOfSmallestDistance = 0;
int16_t angleOffset = 0;

const uint16_t reverseForMS = 2000;
volatile uint32_t reverseUntilTime = 0;

const uint8_t pickupDistance = 5;

void startSearch() {
    angleOffset = angleZ();
    smallestDistanceFound = ultrasonic.MAX_DISTANCE;
    angleOfSmallestDistance = angleOffset;
    motor.setLeftSpeed(-1);
    motor.setRightSpeed(1);

    state = searchState;
}

void startReverse() {
    motor.setLeftSpeed(-1);
    motor.setRightSpeed(-1);
    reverseUntilTime = millis() + reverseForMS;

    state = reverseState;
}

void startApproach() {
    motor.setLeftSpeed(1);
    motor.setRightSpeed(1);

    state = approachState;
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
        uint16_t distance = ultrasonic.get_distance();
        int16_t angle = angleZ();
        if (distance < smallestDistanceFound) {
            smallestDistanceFound = distance;
            angleOfSmallestDistance = angle;
        }

        if (angleZ() - angleOffset >= 360) {
            startApproach();
        }

        // TODO: nothing found?

    } break;

    case approachState: {
        uint16_t distance = ultrasonic.get_distance();

        // lost it again
        if (distance == ultrasonic.MAX_DISTANCE) {
            startSearch();
            return;
        }

        if (distance <= pickupDistance) {
            startSearch();
            return;
        }

    } break;

    case reverseState: {
        if (millis() > reverseUntilTime)
            state = searchState;
    } break;

    case pickupState: {
        // TODO: try to pick it up multiple times
    } break;

    case returnState: {
        // wait for interrupt
    } break;

    case finalState: {
        // done.
    } break;
    }

    // keep constant loop duration
    delay(targetLoopDuration - (lastLoopTime % targetLoopDuration));
}

int16_t angleZ() { return mpu.GetAngZ() + 179; }
