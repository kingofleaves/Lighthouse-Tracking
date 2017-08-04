#include "HomographyPose.h"
#include "WProgram.h"
#include "Utils.h"

void HomographyPose::unitTests() {
  test_updateParameters();
  test_solveForHomography();
  test_solveFor3DPosition();
  test_computeResidual();
}

void HomographyPose::test_updateParameters() {
  float testProj2D[8] = { -0.579502165, -0.462588221, -0.538486242, -0.45121488, -0.548149765, -0.503692448, -0.589576244, -0.517694592};
  float correctA[8][8] = {
    {-42, 25, 1, 0, 0, 0, -24.3390903, 14.4875546 },
    {0, 0, 0, -42, 25, 1, -19.4287052, 11.5647058, },
    {42, 25, 1, 0, 0, 0, 22.6164227, 13.4621563},
    {0, 0, 0, 42, 25, 1, 18.951025, 11.2803717 },
    {42, -25, 1, 0, 0, 0, 23.0222893, -13.7037439 },
    {0, 0, 0, 42, -25, 1, 21.1550827, -12.5923109 },
    {-42, -25, 1, 0, 0, 0, -24.7622032, -14.7394066 },
    {0, 0, 0, -42, -25, 1, -21.7431736, -12.9423647}
  };

  updateParameters(testProj2D);

  Serial.println("Testing HomographyPose::updateParameters()");
  float diffA = Utils::ComputeL2Error((float*)correctA, (float*)A, 8,8);
  Serial.printf("Difference in A: %.5g \n", diffA);
  float diffB = Utils::ComputeL2Error(testProj2D, b, 1,8);
  Serial.printf("Difference in b: %.5g \n", diffB);
}

void HomographyPose::test_solveForHomography() {
  float correcth[8] = {0.000159732997, 0.000218080357, -0.563425064, -0.000132918358, 0.0010930337, -0.483665943, 0.000586956739, -3.68691981e-05};
  float testB[8] = { -0.579502165, -0.462588221, -0.538486242, -0.45121488, -0.548149765, -0.503692448, -0.589576244, -0.517694592};
  float testA[8][8] = {
    {-42, 25, 1, 0, 0, 0, -24.3390903, 14.4875546 },
    {0, 0, 0, -42, 25, 1, -19.4287052, 11.5647058, },
    {42, 25, 1, 0, 0, 0, 22.6164227, 13.4621563},
    {0, 0, 0, 42, 25, 1, 18.951025, 11.2803717 },
    {42, -25, 1, 0, 0, 0, 23.0222893, -13.7037439 },
    {0, 0, 0, 42, -25, 1, 21.1550827, -12.5923109 },
    {-42, -25, 1, 0, 0, 0, -24.7622032, -14.7394066 },
    {0, 0, 0, -42, -25, 1, -21.7431736, -12.9423647}
  };
  memcpy(b, testB, 8*sizeof(float));
  memcpy((float*)A, (float*)testA, 64 * sizeof(float));

  solveForHomography();

  Serial.println("Testing HomographyPose::solveForHomography()");
  float diffh = Utils::ComputeL2Error(correcth, h, 1,8);
  Serial.printf("Difference in h: %.5g \n", diffh);
}

void HomographyPose::test_solveFor3DPosition() {
  float correctPos3D[3] = {-648.419067, -556.628113, -1150.85242};
  float testh[8] = {0.000159732997, 0.000218080357, -0.563425064, -0.000132918358, 0.0010930337, -0.483665943, 0.000586956739, -3.68691981e-05};
  memcpy(h, testh, 8*sizeof(float));

  solveFor3DPosition();

  Serial.println("Testing HomographyPose::solveFor3DPosition()");
  float diff3D = Utils::ComputeL2Error(correctPos3D, position3D, 1,3);
  Serial.printf("Difference in position3D: %.5g \n", diff3D);
}

void HomographyPose::test_computeResidual() {
  float testProjection2D[8] = {-0.579502165, -0.462588221, -0.538486242, -0.45121488, -0.548149765, -0.503692448, -0.589576244, -0.517694592};
  float testNormalizationFactor = 0.00086892116814851761;
  float testh[8] = {0.000159732997, 0.000218080357, -0.563425064, -0.000132918358, 0.0010930337, -0.483665943, 0.000586956739, -3.68691981e-05};
  float correctResidual = 1.34026124e-12;

  memcpy(h, testh, 8*sizeof(float));

  float res = computeResidual(testProjection2D, testNormalizationFactor);

  Serial.println("Testing HomographyPose::computeResidual()");
  float diffResidual = sq(correctResidual - res);
  Serial.printf("Difference in residual: %.5g \n", diffResidual);
}