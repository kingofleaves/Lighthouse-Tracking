#ifndef _LevenbergMarquardt_h_
#define _LevenbergMarquardt_h_

class LevenbergMarquardtPose
{
public:
	// Maximum iterations with which to run the LM optimization.
	static const int kMaxIters = 3;

	// The lambda dampening factor to use.
	const float lambdaLM = 0.1;

	// Displacement of features (in our case photodiodes) from the center
	// of a flat planar object (Vrduino board) that we estimate the pose of.
	// Units are mm. Copied in during construction from photoDiodeLocations.
	// Formatted : [x_0 y_0 x_1 y_1 x_2 y_2 x_3 y_3]
	float object2D[8] = {};

	// 3D pose of the VRduino board center, both rotation and translation.
	// This value gets updated in computePosition() every time a pose is
	// to be estimated. 
	// Formatted : [thetax, thetay, thetaz,x,y,z]
	float pose3D[6] = {};

	// Array of residuals between objective function, f(g(x)), that were
	// induced after each iteration of the LM algorithm.
	float residual[kMaxIters+1] = {};

	// Constructor simply copies photoDiodeLocations into object2D
	LevenbergMarquardtPose(const float* photoDiodeLocations);
	~LevenbergMarquardtPose(){};

	// Evaluates the function h=g(x) at x, as specified on Slide 34
	// of Lecture 12. Maps from rotation/translation to the 
	// homography.
	void eval_g_at_x(const float *x, float *h);

	// Evaluates the function f=g(h) at h, as specified on Slide 34
	// of Lecture 12. Maps from the homography to the x,y coordinates
	// in the light house's "sensor" space to which the diodes get
	// projected onto.
	void eval_f_at_h(const float *h, float *f);

	// Computes the Jacobian of g(x) at the position x.
	void get_jacobian_g(const float *x, float Jg[][6]);

	// Computes the Jacobian of h(x) at the position x.
	void get_jacobian_f(const float *h, float Jf[][9]);

	// Computes the step in which to move following this iteration as 
	// defined in Lecture 12 Slide 43. Returns false if a matrix cannot be
	// inverted. 
	bool compute_delta_x(const float* J, const float* f, const float* projection2D, float* deltaX);

	// Computes the 3D pose of a planar object based on its 2D projection
	// onto a plane a unit distance away from light house, using a guess onto 
	// where it might be relative to the light house. Returns true
	// if the if the computation was a succes, or false if it fails (like the
	// matrix A being non-invertible).
	bool computePosition(const float* projection2D, const float* positionEstimate);

	// Computes residual between measured features on the 2D plane in front 
	// of the lighthouse and the estimated rotation/translation of the object
	// as computed by the LM algorithm.
	float get_residual(const float *x, const float *pos2D);

	// Numeric gradient check of the jacobian of f
	void get_jacobian_f_numeric(const float *h, float Jf[][9]);

	// Numeric gradient check of the jacobian of g
	void get_jacobian_g_numeric(const float *x, float Jg[][6]);

	// Jacobian matrix checker. Compares the algebraic implementations
	// in get_jacobian_g() and get_jacobian_f() to the numberical methods
	// of estimated the jacobians.
	void check_jacobians(void);
	
	// Unit test (defined in TestLMPose.cpp)
	void unit_tests();
	void test_eval_g_at_x();
	void test_eval_f_at_h();
	void test_get_jacobian_f();
	void test_get_jacobian_g();
	void test_compute_delta_x();
	void test_computePosition();


};

#endif