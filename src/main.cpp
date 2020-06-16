#include <Arduino.h>
//#include <TinyMPU6050.h>
#include <Ultrasonic.h>

//MPU6050 mpu(Wire);

Ultrasonic ultrasonic(2, 3);

const uint8_t motorLaPin = 10;
const uint8_t motorLbPin = 9;
const uint8_t motorRaPin = 6;
const uint8_t motorRbPin = 5;

const uint8_t maxSpeed = 100;

const int8_t motorTuning = -5;

void motorL(int8_t d);
void motorR(int8_t d);
int16_t angleZ();

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


uint16_t minDistance = 300;
int16_t minAngle = 0;
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
      minDistance = 300;
      minAngle = angleOffset;
      motorL(-100);
      motorR(100);
      state = searchState;
    }
    break;

    case searchState: {
      uint16_t distance = ultrasonic.read();
      int16_t angle = angleZ();
      if (distance < minDistance) {
        minDistance = distance;
        minAngle = angle;
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
  int speed = 255 - ((int16_t)d * maxSpeed)/100 + motorTuning;
  if (d > 0) {
    // forward
    digitalWrite(motorLaPin, LOW);
    analogWrite(motorLbPin, speed);
  } else if (d < 0) {
    // backward
    analogWrite(motorLaPin, speed);
    digitalWrite(motorLbPin, LOW);
  } else {
    // stop
    digitalWrite(motorLaPin, LOW);
    digitalWrite(motorLbPin, LOW);
  }
}

void motorR(int8_t d) {
  d = constrain(d, -100, 100);
  int speed = 255 - ((int16_t)d * maxSpeed)/100 - motorTuning;
  Serial.println(speed);
  if (d > 0) {
    // forward
    digitalWrite(motorRaPin, LOW);
    analogWrite(motorRbPin, speed);
  } else if (d < 0) {
    // backward
    analogWrite(motorRaPin, speed);
    digitalWrite(motorRbPin, LOW);
  } else {
    // stop
    digitalWrite(motorRaPin, LOW);
    digitalWrite(motorRbPin, LOW);
  }
}

int16_t angleZ() {
  //mpu.GetAngZ()+180;
  return 0;
}
