#include <Arduino.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include "Ultrasonic.h"
#include "Gyro.h"
#include "ManagedMotors.h"
#include "Lights.h"
#include "MagnetSensor.h"
#include "LineSensor.h"

Ultrasonic ultrasonic;
Gyro gyro;
ManagedMotors motors = ManagedMotors(gyro);
Lights lights;
MagnetSensor magnet;
LineSensor line;

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

void startSearch() {
    initial_angle = gyro.getAngleZ();
    smallestDistanceFound = ultrasonic.MAX_DISTANCE;
    angleOfSmallestDistance = initial_angle;

    motors.turn(0.5);

    state = searchState;
}

void startReverse() {
    motors.move(-0.5);
    reverseUntilTime = millis() + reverseForMS;

    state = reverseState;
}

void startAlign() {
    // TODO: find shortest turn direction
    motors.turn(0.1);

    state = alignToTargetState;
}

void startApproach() {
    motors.move(0.1);

    state = approachState;
}

void startPickup() {
    motors.stop();

    state = pickupState;
}

void line_found() {
    DEBUG_PRINTLN("line_found");
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
      motors.move(i);
      delay(500);
    }
  }
}

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    ultrasonic.begin();
    motors.begin();
    lights.begin();
    gyro.begin();
    magnet.begin();
    line.begin();
    
    line.registerListener(line_found);
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
        assert !magnet.detected()
        servo down
        if magnet.detected():
            servo up
            assert magnet.detected()
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

    // keep movement straight
    motors.update();

    // keep constant loop duration
    lights.delay(targetLoopDuration - (lastLoopTime % targetLoopDuration));
}
