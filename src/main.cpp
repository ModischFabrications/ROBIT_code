#include <Arduino.h>
#include <TinyMPU6050.h>
#include <Ultrasonic.h>

MPU6050 mpu(Wire);

Ultrasonic ultrasonic;

const uint8_t motorLPins[2] = {10, 9};
const uint8_t motorRPins[2] = {6, 5};

const uint8_t maxPWMSpeed = 100; // 0 to 255

const int8_t motorTuningLeftToRight = -5;

void motorL(float d); // -1 to 1
void motorR(float d); // -1 to 1
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
        motorL(-100);
        motorR(100);
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
        motorL(100);
        motorR(100);
    } break;

    case pickupState: {

    } break;

    case returnState: {

    } break;

    case finalState: {

    } break;
    }
}

void motorL(float d) {
    d = constrain(d, -1, 1);
    int speed = 255 - d * maxPWMSpeed + motorTuningLeftToRight;
    if (d > 0) {
        // forward
        digitalWrite(motorLPins[0], LOW);
        analogWrite(motorLPins[1], speed);
    } else if (d < 0) {
        // backward
        analogWrite(motorLPins[0], speed);
        digitalWrite(motorLPins[1], LOW);
    } else {
        // stop
        digitalWrite(motorLPins[0], LOW);
        digitalWrite(motorLPins[1], LOW);
    }
}

void motorR(float d) {
    d = constrain(d, -1, 1);
    int speed = 255 - d * maxPWMSpeed - motorTuningLeftToRight;
    Serial.println(speed);
    if (d > 0) {
        // forward
        digitalWrite(motorRPins[0], LOW);
        analogWrite(motorRPins[1], speed);
    } else if (d < 0) {
        // backward
        analogWrite(motorRPins[0], speed);
        digitalWrite(motorRPins[1], LOW);
    } else {
        // stop
        digitalWrite(motorRPins[0], LOW);
        digitalWrite(motorRPins[1], LOW);
    }
}

int16_t angleZ() { return mpu.GetAngZ() + 179; }
