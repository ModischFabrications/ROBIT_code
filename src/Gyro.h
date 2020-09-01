/**
 * \class Gyro
 *
 * \brief Level 2 class for MPU6050 sensor
 *
 * This class is based on MPU6050_tockn.h. It supports reading the z-angle (yaw).
 * Call 'begin' prior to usage to initialise the sensor and set a predefined
 * offset. The function 'update' has to be called in every update loop.
 */

#pragma once

#include <Arduino.h>

#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

class Gyro {
  private:
    // can't move MPU here! Some weird error: "Wire is not a type"

  public:
    void begin() {
        mpu6050.begin();
        delay(100);
        // prevent strange init behaviour
        mpu6050.begin();
        // calculcated from mpu6050.calcGyroOffsets(true);
        mpu6050.setGyroOffsets(-1.40, 0.67, 0.63);
        delay(100);
    }

    void update() {
      mpu6050.update();
    }

    /**
     * Nearly full range of int16. multiple turns possible.
     * Negative values and values larger than 360 are possible!
     * positive values in clockwise direction.
     * */
    int16_t getAngleZ() { return mpu6050.getAngleZ(); }
};
