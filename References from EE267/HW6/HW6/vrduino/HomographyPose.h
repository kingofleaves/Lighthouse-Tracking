#ifndef _HomographyPose_h_
#define _HomographyPose_h_

// Class capable of computing pose estimation via homography
class HomographyPose
{
public:

	// The A matrix in Ah = b equation in Lecture 10 slide 73. This variable
	// is updated every time a pose is to be estimated by the
	// updateParameters() member function.
	float A[8][8] = {};
	
	// The b vector in Ah = b equation in Lecture 10 slide 73. This variable
	// is updated every time a pose is to be estimated by the
	// updateParameters() member function.
	float b[8] = {};

	// Vector filled with homography values. The values are updated in
	// solveForHomography() every time a post is to be estimated.
	// Format: [h11 h12 h13 h21 h22 h23 h31 h32]
	float h[8] = {};

	// Displacement of features (in our case photodiodes) from the center
	// of a flat planar object (Vrduino board) that we estimate the pose of.
	// Units are mm. Copied in during construction from photoDiodeLocations.
	// Formatted : [x_0 y_0 x_1 y_1 x_2 y_2 x_3 y_3]
	float object2D[8] = {};

	// 3D position of the VRduino board center. This value gets updated in
	// computePosition() every time a pose is to be estimated. 
	float position3D[3] = {};

	// Residual of our estimate on the 3D position of the VRduino. Computed
	// in copmuteResidual() and updated in computePosition().
	float residual = 0.0;

	// Enable running average of 3D position estimates.
	bool useRunningAverage = false;

	// Alpha value of running average.
	float alpha = 0.5;

	// Copies photoDiodeLocations into object2D
	HomographyPose(float* photoDiodeLocations);
	~HomographyPose(){};

	// Updates the A and b member variables based on the projection of the 
	// photodiode feature onto the plane, a unit distance away from the light
	// house.
	void updateParameters(float* projection2D);

	// Solves the inverse problem Ah = b, using the values stored in the member
	// variables A and b. Should return false if the matrix A is
	// ill-conditioned and not invertible. Stores the output in h.
	bool solveForHomography();

	// Solves for the 3D position of point by normalizing the translation
	// components of the homography matrix values in member variable h.
	float solveFor3DPosition();

	// Computes L2 residual between known 2D projections of features onto the
	// light house and the projection computed by applying the estimated
	// homography to the set of known features on the 2D object.
	float computeResidual(float* projection2D, float normalizationFactor);

	// Computes the 3D position of a planar object based on its 2D projection
	// onto a plane a unit distance away from light house. Returns true
	// if the if the computation was a succes, or false if it fails (like the
	// matrix A being non-invertible).
	bool computePosition(float* projection2D);


	// Unit tests (defined in TestHomographyPose.cpp)
	void unitTests();
	void test_updateParameters();
	void test_solveForHomography();
	void test_solveFor3DPosition();
	void test_computeResidual();

};

#endif