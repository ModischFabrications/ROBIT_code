#include <Arduino.h>
//#include <TinyMPU6050.h>
#include <Ultrasonic.h>

//MPU6050 mpu(Wire);

Ultrasonic ultrasonic(2, 3);

const uint8_t motorLPins[2] = {10, 9};
const uint8_t motorRPins[2] = {6, 5};

const uint8_t maxSpeed = 100;

const int8_t motorTuningLeftToRight = -5;

void motorL(int8_t d);
void motorR(int8_t d);
int16_t angleZ(); // 0 to 359

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
  //mpu.Initialize();
}

void loop() {
  //mpu.update();

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
    }
    break;

    case searchState: {
      uint16_t distance = ultrasonic.read();
      int16_t angle = angleZ();
      if (distance < smallestDistance) {
        smallestDistance = distance;
        angleOfSmallestDistance = angle;
      }

      if (angleZ()-angleOffset >= 360) {
        state = driveForwardState;
      }

    }
    break;

    case driveForwardState: {
      motorL(100);
      motorR(100);
    }
    break;

    case pickupState: {

    }
    break;

    case returnState: {

    }
    break;

    case finalState: {

    }
    break;
  }
}





void motorL(int8_t d) {
  d = constrain(d, -100, 100);
  int speed = 255 - ((int16_t)d * maxSpeed)/100 + motorTuningLeftToRight;
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

void motorR(int8_t d) {
  d = constrain(d, -100, 100);
  int speed = 255 - ((int16_t)d * maxSpeed)/100 - motorTuningLeftToRight;
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

int16_t angleZ() {
  //mpu.GetAngZ()+179;
  return 0;
}
