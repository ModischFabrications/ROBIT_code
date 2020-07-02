#pragma once

#include <Arduino.h>

//#include <TinyMPU6050.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

class Gyro {
  private:
    // TODO: move MPU here! Some weird error: "Wire is not a type"

  public:
    void begin() {
        mpu6050.begin();

        delay(100);
        // calculcated from mpu6050.calcGyroOffsets(true);
        mpu6050.setGyroOffsets(-2.70, 0.94, -0.40);
        delay(100);
    }

    /**
     *  positive values in clockwise direction
     * */
    int16_t getAngleZ() { return mpu6050.getAngleZ(); }
};
