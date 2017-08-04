/**
 * EE267 Virtual Reality
 * Homework 6
 * Positional Tracking via Pose Estimation
 *
 * Instructor: Gordon Wetzstein <gordon.wetzstein@stanford.edu>,
 *         Robert Konrad <rkkonrad@stanford.edu>,
 *         Hayato Ikoma <hikoma@stanford.edu>,
 *         Keenan Molner <kmolner@stanford.edu>
 *
 * @author Hayato Ikoma <hikoma@stanford.edu>
 * @author Gordon Wetzstein <gordon.wetzstein@stanford.edu>,
 * @copyright The Board of Trustees of the Leland
   Stanford Junior University
 * @version 2017/03/28
 *
 */


/* Our IMU class */
#include "imu.h"

/* Our Quaternion class */
#include "Quaternion.h"

/* Our Euler class */
#include "Euler.h"

#include "PhotoDiodes.h"
#include "HomographyPose.h"
#include "LevenbergMarquardt.h"

/***
 * Variable to set if the measureBias function will be executed. If the bias
 * measurement is executed, the estiamted values will be used for the
 * orientation tracking. If not, the default values of gyrBias will be used.
 */
const bool findBias = false;


/* Measured bias */
double gyrBiasX = 3.36259, gyrBiasY = -0.350597, gyrBiasZ = 0.831812;
double accBiasX = 0.65, accBiasY = -1.81, accBiasZ = 9.60;
double magBiasX = 36.47, magBiasY = 2.84, magBiasZ = -26.17;


/* Integrated gyroscope measurements */
double gyrIntX = 0, gyrIntY = 0, gyrIntZ = 0;

/* Constant for complementary filter */
const double alpha = 0.9;

/* Variables for timing measurements */
double previous_time = 0;


/* Variables for streaming data (euler and quaternion based on complementary
   filter) */
Euler eulerCmp  = Euler();
Quaternion qCmp = Quaternion();
float hPose[3] = {};
float lmPose[6] = {};

/***
 * Imu class instance
 * This class is used to read the measurements.
 */
Imu imu = Imu();

// Defines variables need for positional tracking
PhotoDiodes photodiodes;
HomographyPose homography(photodiodes.locations_);
LevenbergMarquardtPose lm(photodiodes.locations_);
bool useHomographyAsEstimate = true;


/***
 * Helper function and variables to stream data
 * Change the variable streamingMode to switch which variables you would like
 * to stream into the serial communication.
 *
 * Feel free to add other data for debugging.
 */
const int NONE       = 0;
const int QUATERNION = 1;
const int HOMOGRAPHY = 2;
const int LM         = 3;
const int QUATLM     = 4;
const int TICKS      = 5;

int streamingMode = NONE;

void streamData() {
  switch (streamingMode) {
  case NONE:
    break;

  case QUATERNION:
    Serial.printf("QC %f %f %f %f\n",
                  qCmp.q[0], qCmp.q[1], qCmp.q[2], qCmp.q[3]);
    break;

  case HOMOGRAPHY:
    Serial.printf("HM %f %f %f\n",
                  hPose[0], hPose[1], hPose[2]);
    break;

  case LM:
    Serial.printf("LM %f %f %f %f %f %f\n",
                  lmPose[0], lmPose[1], lmPose[2],
                  lmPose[3], lmPose[4], lmPose[5]);
    break;

  case QUATLM:
    Serial.printf("QLM %f %f %f %f %f %f %f %f %f %f\n",
                  qCmp.q[0], qCmp.q[1], qCmp.q[2], qCmp.q[3],
                  lmPose[0], lmPose[1], lmPose[2],
                  lmPose[3], lmPose[4], lmPose[5]);
    break;

  case TICKS:
    for (int i = 0; i < 4; i++){
      Serial.printf("TICKS from sensor %d: %d, %d\n",
                  i, photodiodes.clockTicks_[2*i], photodiodes.clockTicks_[2*i + 1]);
    }
    break;
  }
}

/* Helper function to get the sign of a value */
double sign(double value) {
  return double((value > 0) - (value < 0));
}

/**
 * This function is used to measure the bias and variance of the measurements.
 * The estimation is performed by taking 1000 measurements. This function should
 * be executed by placing Arduino stedy.
 */
void measureBias() {
  gyrBiasX = 0, gyrBiasY = 0, gyrBiasZ = 0;
  accBiasX = 0, accBiasY = 0, accBiasZ = 0;
  magBiasX = 0, magBiasY = 0, magBiasZ = 0;

  double gyrVarX = 0, gyrVarY = 0, gyrVarZ = 0;
  double accVarX = 0, accVarY = 0, accVarZ = 0;
  double magVarX = 0, magVarY = 0, magVarZ = 0;

  /* Number of measurements */
  int N = 1000;

  /* Find the mean of measurements */
  for (int i = 0; i < N; ++i) {
    /* read IMU data */
    imu.read();

    gyrBiasX += imu.gyrX / N;
    gyrBiasY += imu.gyrY / N;
    gyrBiasZ += imu.gyrZ / N;

    accBiasX += imu.accX / N;
    accBiasY += imu.accY / N;
    accBiasZ += imu.accZ / N;

    magBiasX += imu.magX / N;
    magBiasY += imu.magY / N;
    magBiasZ += imu.magZ / N;
  }

  /* Find the variance of measurements */
  for (int i = 0; i < N; ++i) {
    /* read IMU data */
    imu.read();

    gyrVarX += sq(imu.gyrX - gyrBiasX) / N;
    gyrVarY += sq(imu.gyrY - gyrBiasY) / N;
    gyrVarZ += sq(imu.gyrZ - gyrBiasZ) / N;

    accVarX += sq(imu.accX - accBiasX) / N;
    accVarY += sq(imu.accY - accBiasY) / N;
    accVarZ += sq(imu.accZ - accBiasZ) / N;

    magVarX += sq(imu.magX - magBiasX) / N;
    magVarY += sq(imu.magY - magBiasY) / N;
    magVarZ += sq(imu.magZ - magBiasZ) / N;
  }

  Serial.printf("X bias: g: %f, m: %f, a: %f\n",
                gyrBiasX, magBiasX, accBiasX);
  Serial.printf("Y bias: g: %f, m: %f, a: %f\n",
                gyrBiasY, magBiasY, accBiasY);
  Serial.printf("Z bias: g: %f, m: %f, a: %f\n",
                gyrBiasZ, magBiasZ, accBiasZ);

  Serial.printf("X variance: g: %f, m: %f, a: %f\n",
                gyrVarX, magVarX, accVarX);
  Serial.printf("Y variance: g: %f, m: %f, a: %f\n",
                gyrVarY, magVarY, accVarY);
  Serial.printf("Z variance: g: %f, m: %f, a: %f\n",
                gyrVarZ, magVarZ, accVarZ);
}

/* Set up Arduino */
void setup() {
  /* Initialize serial communication */
  delay(1000);
  Serial.begin(115200);
  delay(100);
  Serial.println("Serial communication started...");
  delay(1000);

  /* initialize IMU */
  imu.init();

  if (imu.communication) {
    Serial.print("Valid communication with IMU!\n\n");
  } else {
    Serial.print("Invalid communicaiton with IMU...\n\n");
  }

  //print off available streaming modes
  Serial.printf("Streaming modes: \n NONE = 0 \n QUATERNION = 1 \n HOMOGRAPHY = 2 \n LM = 3 \n QUAT and LM = 4 \n TICKS = 5 \n \n");

  // initialze positional tracking
  photodiodes.initTracking();

  // Print which base station we're using
  photodiodes.printBaseStation();

  photodiodes.unitTest_update2DPositions();
  homography.unitTests();
  lm.unit_tests();

  /* Measure bias */
  if (findBias) measureBias(); delay(1000);
}

/* Main loop, read and display data */
void loop() {
  // ORIENTATION TRACKING
  /* Reset the estimation if there is a keyboard input. */
  if (Serial.available()) {
    streamingMode = Serial.parseInt();

    gyrIntX = 0;
    gyrIntY = 0;
    gyrIntZ = 0;

    eulerCmp = Euler();

    qCmp = Quaternion();
  }

  /* Get current time in milliseconds */
  double current_time = millis();

  if (previous_time == 0) {
    previous_time = current_time;
  }

  /* Compute the elapsed time from the previous iteration */
  double deltaT = (current_time - previous_time) / 1000.0;
  previous_time = current_time;

  /* read IMU data */
  imu.read();

  /* remove bias from the gyro measurements */
  double gyrX = imu.gyrX - gyrBiasX;
  double gyrY = imu.gyrY - gyrBiasY;
  double gyrZ = imu.gyrZ - gyrBiasZ;


  double l = sqrt(sq(gyrX) + sq(gyrY) + sq(gyrZ));

  Quaternion qDelta = Quaternion();

  if (l >= 1e-8) {
    qDelta.setFromAngleAxis(
      deltaT * l, gyrX / l, gyrY / l, gyrZ / l);
  }


  /* compute quaternion representing tilt of sensor */
  qCmp = Quaternion().multiply(qCmp, qDelta).normalize();

  /* compute complementary filter */
  Quaternion qa = Quaternion(0, imu.accX, imu.accY, imu.accZ);

  Quaternion qa_inertial = qa.rotate(qCmp);

  double phi = acos(qa_inertial.q[2] / qa_inertial.length()) * RAD_TO_DEG;

  double n = sqrt(sq(qa_inertial.q[1]) + sq(qa_inertial.q[3]));

  Quaternion qTilt = Quaternion().setFromAngleAxis(
    (1 - alpha) * phi, -qa_inertial.q[3] / n, 0.0, qa_inertial.q[1] / n);

  qCmp = Quaternion().multiply(qTilt, qCmp).normalize();


  // POSITIONAL TRACKING
  // Updates photodiode timings and checks if all were updated
  bool updateSuccess = photodiodes.updateClockTicks();
  if (!updateSuccess)
  {
    return;
  }

  // Updates 2D projections onto unit plane with updated clock tick values
  photodiodes.update2DPositions();
  float* projection2D = photodiodes.projection2D_;

  // Computes the 3D position of the VRduino via the homography method. The
  // 3D position is stored internally in the homography object.
  bool bSuccess_Homography = homography.computePosition(projection2D);
  if(bSuccess_Homography) {
    memcpy(hPose, homography.position3D, 3*sizeof(float));
  }

  // Worst case scenario for guessing LM starting condition
  float lmPoseGuess[6] = {0.0,0.0,0.0,0.0,0.0,-2000.0};

  // If homography method was a success and it hasn't been used to 
  // initialize yet then use it as the starting point for LM
  if (bSuccess_Homography && useHomographyAsEstimate){
    lmPoseGuess[3] = homography.position3D[0];
    lmPoseGuess[4] = homography.position3D[1];
    lmPoseGuess[5] = homography.position3D[2];
    useHomographyAsEstimate = false;
  // Use the previous LM 3D pose output as the starting guess for this 
  // iteration
  } else if (bSuccess_Homography) {
    memcpy(lmPoseGuess, lm.pose3D, 6*sizeof(float));
  }

  // Compute the 3D pose of the VRduino via the Levenberg-Marquardt method.
  // The computed 3D pose is stored internally in the LM object.
  bool bSuccess_LM = lm.computePosition(projection2D, lmPoseGuess);
  if(bSuccess_LM) {
    memcpy(lmPose, lm.pose3D, 6*sizeof(float));
  }
  streamData();
}
