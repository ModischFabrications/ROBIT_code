#include <Arduino.h>
#include <TinyMPU6050.h>

#include "Ultrasonic.h"
#include "Motors.h"

MPU6050 mpu(Wire);

Ultrasonic ultrasonic;
Motors motor;

int16_t angleZ();     // 0 to 359

enum FSMstates {
    initState,
    startSearchState,
    searchState,
    driveForwardState,
    pickupState,
    returnState,
    finalState
};

FSMstates state = initState;

uint16_t smallestDistance = 300;
int16_t angleOfSmallestDistance = 0;
int16_t angleOffset = 0;

void setup() {
    Serial.begin(115200);
    mpu.Initialize();
    // mpu.Calibrate();
}

void loop() {
    mpu.Execute();

    switch (state) {
    case initState: {
        state = startSearchState;
        break;

    case startSearchState:
        angleOffset = angleZ();
        smallestDistance = 300;
        angleOfSmallestDistance = angleOffset;
        motor.setLeftSpeed(-1);
        motor.setRightSpeed(1);
        state = searchState;
    } break;

    case searchState: {
        uint16_t distance = ultrasonic.get_distance();
        int16_t angle = angleZ();
        if (distance < smallestDistance) {
            smallestDistance = distance;
            angleOfSmallestDistance = angle;
        }

        if (angleZ() - angleOffset >= 360) {
            state = driveForwardState;
        }

    } break;

    case driveForwardState: {
        motor.setLeftSpeed(1);
        motor.setRightSpeed(1);
    } break;

    case pickupState: {

    } break;

    case returnState: {

    } break;

    case finalState: {

    } break;
    }
}

int16_t angleZ() { return mpu.GetAngZ() + 179; }
