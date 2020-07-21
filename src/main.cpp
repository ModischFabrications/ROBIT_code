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
// enumerated to compare against real output
enum FSMstates : uint8_t {
    initState = 0,
    searchState = 1,
    alignToTargetState = 2,
    approachState = 3,
    adjustState = 4,
    reverseState = 5,
    pickupState = 6,
    returnState = 7,
    finalState = 8
};

volatile FSMstates state = initState;

const uint8_t LED_ERR = 9;

const uint8_t LED_HB = 8;
const CRGB color_HB = CRGB::Orange;

const uint16_t targetLoopDuration = 10;
const uint8_t HeartbeatsPerMinute = 60;

uint16_t smallestDistanceFound = 300;
int16_t angleOfSmallestDistance = 0;
int16_t initial_angle = 0;
uint16_t distanceAtLost = 0;
bool adjustingClockwise = false;
const int16_t adjustmentAngle = 45;

const uint16_t reverseForMS = 1000;
volatile uint32_t reverseUntilTime = 0;

const uint8_t pickupDistance = 5;

void (*restart)(void) = 0;

void startSearch() {
    initial_angle = gyro.getAngleZ();
    smallestDistanceFound = Sonar::MAX_DISTANCE;
    angleOfSmallestDistance = initial_angle;

    motors.turn(0.1);

    state = searchState;
}

void startReverse() {
    motors.move(-0.5);
    reverseUntilTime = millis() + reverseForMS;

    state = reverseState;
}

void startAlign() {
    // TODO: find shortest turn direction (see #42)
    motors.turn(-0.1);

    state = alignToTargetState;
}

void startApproach() {
    distanceAtLost = smallestDistanceFound;
    motors.move(0.1);

    state = approachState;
}

void startAdjust() {
    // TODO: evaluate if moving a bit closer actually helps that much
    motors.move(0.1);
    delay(500);
    motors.stop();
    adjustingClockwise = false;
    // update to current angle
    angleOfSmallestDistance = gyro.getAngleZ();
    // begin by turning counterclockwise
    motors.turn(-0.1);
    state = adjustState;
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
    DEBUG_PRINTLN("We did it!");
}

void line_found() {
    if (state == finalState)
        return;
    if (state == returnState) {
        startFinal();
        // done!
        return;
    }

    // assume we only hit the line on forward movements
    startReverse();
}

void showState(FSMstates state) {
    fill_solid(lights.leds, (uint8_t)finalState, CRGB::Black);
    lights.leds[(uint8_t)state] = CRGB::Blue;
    FastLED.show();
}

void showError(const CRGB color, const __FlashStringHelper* msg) {
    lights.leds[LED_ERR] = color;
    DEBUG_PRINT("[Error] ");
    DEBUG_PRINTLN(msg);
    // wait for the user to see it
    delay(10000);
    restart();
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
    line.begin(false);

    line.registerListener(line_found);

    // prevent motors from spinning on startup
    motors.stop();

    lights.helloPower();

    gyro.update();
    // movement detected without actually moving is a sign of wrong gyro startup
    if (gyro.getAngleZ() != 0)
        restart(); // software reset

    // clear ultrasonic sensor
    servo.moveUp();
    servo.waitUntilStopped();

    if (magnet.detected())
        showError(CRGB::Brown, F("magnet still attached"));
    if (line.detected())
        showError(CRGB::Red, F("starting on line"));
}

void loop() {
    gyro.update();
    servo.update();

    showState(state);

    switch (state) {
    case initState: {
        // waiting until servo reached upper position, timeout set by library
        if (servo.isStopped())
            startSearch();
    } break;

    case searchState: {
        uint8_t current_distance = sonar.get_min_distance();
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
                startReverse();
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

        // anything else can't be our treasure and needs to be ignored
        if (distance > smallestDistanceFound + 10) {
            DEBUG_PRINT("lost it at ");
            DEBUG_PRINTLN(distanceAtLost);
            startAdjust();
            return;
        }

        if (distance <= pickupDistance) {
            // reached it
            startPickup();
            return;
        }

        // decrease distance while searching
        distanceAtLost = distance;

    } break;

    case adjustState: {
        uint16_t current_distance = sonar.get_min_distance();
        int16_t current_angle = gyro.getAngleZ();

        if (current_distance < distanceAtLost + 10) {
            DEBUG_PRINT("found it again at ");
            DEBUG_PRINTLN(current_distance);
            startApproach();
            return;
        }

        if (!adjustingClockwise) {
            if (current_angle <= angleOfSmallestDistance - adjustmentAngle) {
                // not found on the left side, try the right one
                adjustingClockwise = true;
                motors.turn(0.1);
            }
        } else {
            if (current_angle >= angleOfSmallestDistance + adjustmentAngle) {
                // not found
                DEBUG_PRINTLN("Could not find it again, restarting search");
                startSearch();
            }
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
                    startReverse();
                }
            }
        }
    } break;

    case returnState: {
        // wait for interrupt, delay is applied by constant loop time
    } break;

    case finalState: {
        // done, show how happy you are
        fill_rainbow(lights.leds, lights.N_LEDS, beatsin16(20, 0, 359));
        FastLED.show();
        // prevent heartbeat and other controls
        return;
    } break;
    }

    // keep movement straight
    motors.update();

    // fade between black and orange to look like a heartbeat
    CRGB curr_color = blend(CRGB::Black, color_HB, beatsin8(HeartbeatsPerMinute, 0, 255));
    lights.leds[LED_HB] = curr_color;
    FastLED.show();

    // keep constant loop duration by aligning to target duration
    lights.delay(targetLoopDuration - (millis() % targetLoopDuration));
}
