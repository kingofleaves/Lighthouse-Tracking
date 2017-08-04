/**
 * Header file for our IMU class
 *
 * @author Gordon Wetzstein <gordon.wetzstein@stanford.edu>
 * @copyright The Board of Trustees of the Leland
   Stanford Junior University
 * @version 2017/03/28
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

/* for I2C and serial communication */
#include <Wire.h>

class Imu {
public:

  double gyrX, gyrY, gyrZ;
  double accX, accY, accZ;
  double magX, magY, magZ;

  bool communication;

  /* initialize imu */
  void init();

  /* read imu data */
  void read();

  /* hardware connection check */
  void checkCommunication();

private:

  void I2Cread(
    uint8_t  Address,
    uint8_t  Register,
    uint8_t  Nbytes,
    uint8_t *Data);


  void I2CwriteByte(
    uint8_t Address,
    uint8_t Register,
    uint8_t Data);

  uint8_t readByte(uint8_t address,
                   uint8_t readRegister);

  /* adjustment value for magnetometer */
  double _magnetometerAdjustmentScaleX,
         _magnetometerAdjustmentScaleY,
         _magnetometerAdjustmentScaleZ;
};

#endif // ifndef IMU_H
