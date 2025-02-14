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

typedef void (*async)();
volatile async async_call = nullptr;

const uint8_t LED_HB = 8;
const uint8_t LED_ERR = 9;

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

void showState(FSMstates state) {
    // clear old all old values to prevent undefined states
    fill_solid(lights.leds, (uint8_t)finalState, CRGB::Black);
    lights.leds[(uint8_t)state] = CRGB::Blue;
    FastLED.show();
}

/**
 * Use this to schedule complex calls from ISRs
 * */
void callAsync(async function) {
    async_call = function;
}

void setState(FSMstates new_state) {
    state = new_state;
    showState(state);
}

void showError(const CRGB color, const __FlashStringHelper* msg) {
    lights.leds[LED_ERR] = color;
    DEBUG_PRINT("[Error] ");
    DEBUG_PRINTLN(msg);
    motors.stop();
    // prevent interrupt
    state = initState;
    // wait for the user to see it
    delay(10000);
    restart();
}

void startSearch() {
    initial_angle = gyro.getAngleZ();
    smallestDistanceFound = Sonar::MAX_DISTANCE;
    angleOfSmallestDistance = initial_angle;

    motors.turn(0.1);

    setState(searchState);
}

void startReverse() {
    // reset in case a line was detected while picking up
    servo.moveUp();

    motors.move(-0.5);
    reverseUntilTime = millis() + reverseForMS;

    setState(reverseState);
}

void startAlign() {
    motors.turn(-0.1);

    setState(alignToTargetState);
}

void startApproach() {
    distanceAtLost = smallestDistanceFound;
    motors.move(0.1);

    setState(approachState);
}

void startAdjust() {
    // moving a bit closer helps finding it again
    motors.move(0.1);
    delay(500);
    motors.stop();
    adjustingClockwise = false;
    // update to current angle
    angleOfSmallestDistance = gyro.getAngleZ();
    // begin by turning counterclockwise
    motors.turn(-0.1);
    setState(adjustState);
}

void startPickup() {
    motors.stop();
    servo.moveDown();
    setState(pickupState);
}

void startReturn() {
    motors.move(0.5);
    setState(returnState);
}

void startFinal() {
    line.removeListener();
    motors.stop();
    setState(finalState);
    DEBUG_PRINTLN("We did it!");
}

void line_found() {
    // don't do anything expensive here or this will crash the processor
    if (state == finalState || state == reverseState || state == initState)
        return;
    if (state == returnState) {
        callAsync(startFinal);
        // done!
        return;
    }

    // assume we only hit the line on forward movements
    // this won't look for a better target but at least it prevents runoffs
    callAsync(startReverse);
}

void showDistance(uint8_t distance) {
    const uint8_t max_scale = 255;
    uint8_t rel_distance = ((float)distance / sonar.MAX_DISTANCE) * max_scale;
    // DEBUG_PRINTLN(rel_distance);
    // nice blending with diverse colors
    CRGB new_color = blend(CRGB::DarkGreen, CRGB::SteelBlue, rel_distance);
    // adjust brightness to improve perception of scale
    new_color.nscale8_video((max_scale-rel_distance)/2);
    // 0 is unused, better than hiding currently active state
    lights.leds[0] = new_color;
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
    line.begin(true);

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
    // keep movement straight
    gyro.update();
    motors.update();
    servo.update();

    switch (state) {
    case initState: {
        // waiting until servo reached upper position, timeout set by library
        if (servo.isStopped())
            startSearch();
    } break;

    case searchState: {
        uint8_t current_distance = sonar.get_min_distance();
        showDistance(current_distance);
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
        uint16_t current_distance = sonar.get_min_distance();
        showDistance(current_distance);

        // anything else can't be our treasure and needs to be ignored
        if (current_distance > smallestDistanceFound + 10) {
            DEBUG_PRINT("lost it at ");
            DEBUG_PRINTLN(distanceAtLost);
            startAdjust();
            return;
        }

        if (current_distance <= pickupDistance) {
            // reached it
            startPickup();
            return;
        }

        // decrease distance while searching
        distanceAtLost = current_distance;

    } break;

    case adjustState: {
        uint16_t current_distance = sonar.get_min_distance();
        showDistance(current_distance);
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

    // async setter for ISRs
    if (async_call != nullptr) {
        DEBUG_PRINTLN("Calling async function");
        async_call();
        async_call = nullptr;
    }

    // fade between black and orange to look like a heartbeat; don't use max brightness
    CRGB curr_color = blend(CRGB::Black, CRGB::DarkOrange, beatsin8(HeartbeatsPerMinute, 0, 150));
    lights.leds[LED_HB] = curr_color;
    FastLED.show();

    // keep constant loop duration by aligning to target duration
    lights.delay(targetLoopDuration - (millis() % targetLoopDuration));
}
