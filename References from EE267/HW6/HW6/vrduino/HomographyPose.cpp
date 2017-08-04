#include "HomographyPose.h"
#include "MatrixMath.h"
#include "Utils.h"

// inputs : photoDiodeLocations - displacements of photodiodes relative to center
HomographyPose::HomographyPose(float* photoDiodeLocations) {
  memcpy(object2D, photoDiodeLocations, 8 * sizeof(float));
}

// Updates A and b member variables based on new feature projection information
// inputs : projection2D - 1x8 array of the x,y projection of each diode
//                         onto the light house "sensor" plane
void HomographyPose::updateParameters(float* projection2D) {

}

// inputs : A,b member variables
// output : h member variable
bool HomographyPose::solveForHomography() {
  //MatrixMath::Matrix;
  int a = MatrixMath::Invert((float *) A, 8);
  return true;
}

// inputs : h member variable
// outputs: normalization factor - normalization factor to reduce the columnns
//                                 of the rotations of the homography to sum
//                                 to one.
float HomographyPose::solveFor3DPosition() {
  float normalizationFactor = 0.0;

  // position3D[0] = ...
  // position3D[1] = ...
  // position3D[2] = ...
  
  return normalizationFactor;
}

// Computes the 3D position of an object based on the photodiode features
// projected onto a plane a unit distance from the light house. The function
// returns false if it is not able to compute the inverse of the marix A in
// solveForHomography().
// inputs : projection2D - 1x8 array of the x,y projection of each diode
//                         onto the light house "sensor" plane
// outputs : position3D member variable
bool HomographyPose::computePosition(float* projection2D) {
  bool success = true;
  
  return success;
}

// inputs : projection2D        - 1x8 array of the x,y projection of each diode
//                                onto the light house "sensor" plane
//          normalizationFactor - normalization factor to reduce the columnns
//                                of the rotations of the homography to sum
//                                to one.
// 
// outputs : res                - residual computed with L2 loss metric
float HomographyPose::computeResidual(float* projection2D, float normalizationFactor) {
  float res = 0.0;

  return res;
}
