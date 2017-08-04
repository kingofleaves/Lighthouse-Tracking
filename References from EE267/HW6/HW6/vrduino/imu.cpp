/**
 * IMU class
 *
 * @author Gordon Wetzstein <gordon.wetzstein@stanford.edu>
 * @copyright The Board of Trustees of the Leland
   Stanford Junior University
 * @version 2017/05/07
 */


#include "imu.h"

/*  address of gyro & accelerometer */
#define MPU9250_ADDRESS 0x68

/* Register address with expected 0x71 value when queried. */
#define WHO_AM_I_MPU9250 0x75 // address for expected return value

/* expected value to be returned */
#define MPU9250_KNOWN_VAL 0x71

/*  address of magnetometer (separate chip) */
#define MAG_ADDRESS 0x0C

/***
 * gyro maximum angular velocity range (in degrees per second)
 * note: smaller range makes the measurements more precise with the 16 bit ADC,
 * but is problematic for faster motions
 */
#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

/***
 * accelerometer maximum range (in g, 1 g = 9.81 m/s^2)
 * note: smaller range makes the measurements more precise with the 16 bit ADC,
 *        but is problematic for faster accelerations
 */
#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18


void Imu::init()
{
  Wire.begin();
  delay(2000);
  checkCommunication();

  
  // Configure gyroscope range (use maximum range)
  this->I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);

  // Configure accelerometers range (use maximum range)
  this->I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  // Set bypass mode for the magnetometer, so we can read values directly
  this->I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // read adjument value
  uint8_t buf[3];
  this->I2Cread(MAG_ADDRESS, 0x10, 3, buf);
  this->_magnetometerAdjustmentScaleX = 0.5 * (double(buf[0]) - 128) / 128 + 1;
  this->_magnetometerAdjustmentScaleY = 0.5 * (double(buf[1]) - 128) / 128 + 1;
  this->_magnetometerAdjustmentScaleZ = 0.5 * (double(buf[2]) - 128) / 128 + 1;

  // Request first magnetometer single 16 bit measurement
  this->I2CwriteByte(MAG_ADDRESS, 0x0A, B00010001);

  //  uint8_t cntl_reg;
  //  I2Cread(MAG_ADDRESS,0x0A,1,&cntl_reg);
  //  Serial.println (cntl_reg, BIN);
}

/***
 * This function reads Nbytes bytes from I2C device at address Address.
 * Put read bytes starting at register Register in the Data array.
 */
void Imu::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes,
                  uint8_t *Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;

  while (Wire.available()) Data[index++] = Wire.read();
}

/* Write a byte (Data) in device (Address) at register (Register) */
void Imu::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

/***
 *  read all 9 sensors from the IMU and convert values into metric units
 *  note: these values will be reported in the coordinate system of the sensor,
 *        which may be different for gyro, accelerometer, and magnetometer
 */
void Imu::read() {
  // all measurements are converted to 16 bits by the IMU-internal ADC
  double max16BitValue = 32767.0;


  uint8_t Buf[14];

  this->I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  /* Read accelerometer */

  /* 16 bit accelerometer data */
  int16_t ax = Buf[0] << 8 | Buf[1];
  int16_t ay = Buf[2] << 8 | Buf[3];
  int16_t az = Buf[4] << 8 | Buf[5];

  /* scale to get metric data in m/s^2 */

  // float maxAccRange   = 2.0; // in g
  double maxAccRange = 16.0;                             // max range (in g)
                                                         // as set in setup()
                                                         // function
  double g2ms2    = 9.80665;
  double accScale = g2ms2 * maxAccRange / max16BitValue; // convert 16 bit to
                                                         // float

  /* convert 16 bit raw measurement to metric float */
  accX = - double(ax) * accScale;
  accY = double(ay) * accScale;
  accZ = - double(az) * accScale;


  /* Read gyroscope */

  /* 16 bit gyroscope raw data */
  int16_t gx = Buf[8] << 8 | Buf[9];
  int16_t gy = Buf[10] << 8 | Buf[11];
  int16_t gz = Buf[12] << 8 | Buf[13];

  double maxGyrRange = 2000.0;                   // max range (in deg per sec)
                                                 // as set in setup() function
  // float maxGyrRange = 500.0;
  double gyrScale = maxGyrRange / max16BitValue; // convert 16 bit to float

  /* convert 16 bit raw measurement to metric float */
  gyrX = - double(gx) * gyrScale;
  gyrY = double(gy) * gyrScale;
  gyrZ = - double(gz) * gyrScale;


  /* Read magnetometer */
  uint8_t ST1;
  I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);

  /* new measurement available (otherwise just move on) */
  if (ST1 & 0x01) {
    /* Read magnetometer data */
    uint8_t m[6];
    I2Cread(MAG_ADDRESS, 0x03, 6, m);

    /***
     *  see datatsheet:
     *  - byte order is reverse from other sensors
     *  - x and y are flipped
     *  - z axis is reverse
     */
    int16_t mmy =  m[1] << 8 | m[0];
    int16_t mmx =  m[3] << 8 | m[2];
    int16_t mmz = -m[5] << 8 | m[4];

    /* convert 16 bit raw measurement to metric float */
    double magScale = 4912.0 / max16BitValue;
    this->magX = double(mmx) * magScale * this->_magnetometerAdjustmentScaleX;
    this->magY = double(mmy) * magScale * this->_magnetometerAdjustmentScaleY;
    this->magZ = double(mmz) * magScale * this->_magnetometerAdjustmentScaleZ;

    /* request next reading on magnetometer */
    I2CwriteByte(MAG_ADDRESS, 0x0A, B00010001);
  }
}

void Imu::checkCommunication()
{
  Serial.println("Checking IMU Communication");
  byte imuReturnVal = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  this->communication = (imuReturnVal == MPU9250_KNOWN_VAL);
}

// keenan included readByte to return a single byte over I2C
uint8_t Imu::readByte(uint8_t address, uint8_t readRegister)
{
  uint8_t data;                          // `data` will store the register data

  Wire.beginTransmission(address);       // Initialize the Tx buffer
  Wire.write(readRegister);              // Put slave register address in Tx
                                         // buffer
  Wire.endTransmission(false);           // Send the Tx buffer, but send a
                                         // restart to keep connection alive
  Wire.requestFrom(address, (uint8_t)1); // Read one byte from slave register
                                         // address
  data = Wire.read();                    // Fill Rx buffer with result
  return data;                           // Return data read from slave register
}
