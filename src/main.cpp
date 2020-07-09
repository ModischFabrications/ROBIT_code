#include <Arduino.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include "Gyro.h"
#include "Lights.h"
#include "LineSensor.h"
#include "MagnetSensor.h"
#include "ManagedMotors.h"
#include "ManagedServo.h"
#include "Sonar.h"
#include "Ultrasonic.h"

Sonar sonar;
Gyro gyro;
ManagedMotors motors = ManagedMotors(gyro);
ManagedServo servo;
Lights lights;
MagnetSensor magnet;
LineSensor line;

// keep smaller than N_LEDs for printout!
enum FSMstates : uint8_t {
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
const uint8_t HeartbeatsPerMinute = 60;

uint16_t smallestDistanceFound = 300;
int16_t angleOfSmallestDistance = 0;
int16_t initial_angle = 0;

const uint16_t reverseForMS = 1000;
volatile uint32_t reverseUntilTime = 0;

const uint8_t pickupDistance = 5;

void(* restart) (void) = 0;

void startSearch() {
    initial_angle = gyro.getAngleZ();
    smallestDistanceFound = Sonar::MAX_DISTANCE;
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
    motors.turn(-0.1);

    state = alignToTargetState;
}

void startApproach() {
    motors.move(0.1);

    state = approachState;
}

void startPickup() {
    motors.stop();
    servo.moveDown();
    state = pickupState;
}

void startReturn() {
    motors.move(0.5);
    state = returnState;
}

void startFinal() {
    motors.stop();
    state = finalState;
}

void line_found() {
    DEBUG_PRINTLN("line_found");
    if (state == finalState) return;
    if (state == returnState) {
        startFinal();
        // done!
        return;
    }

    // assume we only hit the line on forward movements
    startReverse();
}

void driveTest() {
    while (1) {
        float i = -1;
        for (; i < 1; i += 0.1) {
            motors.move(i);
            delay(500);
        }
    }
}

void showState(FSMstates state) {
    DEBUG_PRINT("state: ");
    DEBUG_PRINTLN(state);

    fill_solid(lights.leds, lights.N_LEDS - 1, CRGB::Black);
    lights.leds[(uint8_t)state] = CRGB::Blue;
    FastLED.show();
}

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    sonar.begin();
    motors.begin();
    servo.begin();
    lights.begin();
    gyro.begin();
    magnet.begin();
    line.begin();

    line.registerListener(line_found);
    // driveTest();
    // prevent motors from spinning on startup
    motors.stop();

    lights.helloPower();

    gyro.update();
    // movement detected without actually moving is a sign of wrong gyro startup
    if (gyro.getAngleZ() != 0) restart(); // software reset

    // clear ultrasonic sensor
    servo.moveUp();
}

void loop() {
    gyro.update();
    servo.update();

    showState(state);

    switch (state) {
    case initState: {
      // waiting until servo reached upper position, timeout set by library
      if (servo.isStopped()) startSearch();
    } break;

    case searchState: {
        uint16_t current_distance = sonar.get_distance();
        int16_t current_angle = gyro.getAngleZ();
        if (current_distance < smallestDistanceFound) {
            // found something closer
            smallestDistanceFound = current_distance;
            angleOfSmallestDistance = current_angle;
        }
        if (current_angle - initial_angle >= 360) {
            // full rotation
            if (smallestDistanceFound == Sonar::MAX_DISTANCE) {
                // nothing found
                // TODO: random move to new search position
                return;
            }
            startAlign();
        }

    } break;

    case alignToTargetState: {
        // turn to face shortest distance
        // approximated comparison not actually necessary right now
        if (gyro.getAngleZ() <= angleOfSmallestDistance) {
            startApproach();
        }

    } break;

    case approachState: {
        uint16_t distance = sonar.get_min_distance();

        if (distance == Sonar::MAX_DISTANCE) {
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
      // wait for movement down from transition
      if (servo.isStopped()) {
        servo.moveUp();
        if (servo.isStopped()) {
          if (magnet.detected()) {
            // found!
            startReturn();
          } else {
            // TODO: maybe reverse first
            startSearch();
          }
        }
      }
    } break;

    case returnState: {
        // wait for interrupt
        // delay() won't affect the ISR
        //delay(10);
    } break;

    case finalState: {
        // done.
        //FastLED.showColor(CRGB::White);
        // rainbow!
        fill_rainbow(lights.leds, lights.N_LEDS, beatsin16(20, 0, 359));

        //delay(1000);
    } break;
    }

    // keep movement straight
    motors.update();

    // fade between black and orange to look like a heartbeat
    CRGB curr_color = blend(CRGB::Black, CRGB::Orange, beatsin8(HeartbeatsPerMinute, 0, 255));
    lights.leds[lights.N_LEDS - 1] = curr_color;
    FastLED.show();

    // keep constant loop duration by aligning to target duration
    lights.delay(targetLoopDuration - (millis() % targetLoopDuration));
}
