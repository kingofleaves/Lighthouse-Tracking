#include "LevenbergMarquardt.h"
#include "MatrixMath.h"
#include "Utils.h"

// inputs : photoDiodeLocations - 1x8 array of displacements of photodiodes
// 								  relative to center in mm. 
LevenbergMarquardtPose::LevenbergMarquardtPose(const float* photoDiodeLocations) {
	memcpy(object2D, photoDiodeLocations, 8 * sizeof(float));
}

// Given x, computes h as h=g(x). Maps angles/positions to homography values.
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
// outputs: h - 1x9 array of homography values
// 				[h1 h2 h3 h4 h5 h6 h7 h8 h9]
void LevenbergMarquardtPose::eval_g_at_x(const float *x, float *h)
{
	float ox = x[0];
	float oy = x[1];
	float oz = x[2];
	float tx = x[3];
	float ty = x[4];
	float tz = x[5];
	float[] h = {
		cos(oy)*cos(oz)-sin(ox)*sin(oy)*sin(oz),
		-cos(ox)*sin(oz),
		tx,
		cos(oy)*sin(oz)+sin(ox)*sin(oy)*cos(oz),
		cos(ox)*cos(oz),
		ty,
		cos(ox)*sin(oy),
		-sin(ox),
		-tz
	}
	return h;
}

// Given h, computes f as f=g(h). Map from homography values to x,y coordinates
// of each of the photodiodes on the light house "camera"
// inputs: h - 1x9 array of homography values
// 			   [h1 h2 h3 h4 h5 h6 h7 h8 h9]
// outputs: f - 1x8 array of estimated x,y coordinates of all 4 photodiodes
//			    [x1 y1 x2 y2 ... x4 y4]
void LevenbergMarquardtPose::eval_f_at_h(const float *h, float *f)
{
	h1 = h[0]; h2 = h[1]; h3 = h[2]; h4 = h[3]; h5 = h[4]; h6 = h[5]; h7 = h[6]; h8 = h[7]; h9 = h[8];
	p1x = object2D[0]; p1y = object2D[1];
	p2x = object2D[2]; p2y = object2D[3];
	p3x = object2D[4]; p3y = object2D[5];
	p4x = object2D[6]; p4y = object2D[7];
	float[] f = {
		(h1*p1x+h2*p1y+h3)/(h7*p1x+h8*p1y+h9),
		(h4*p1x+h5*p1y+h6)/(h7*p1x+h8*p1y+h9),
		(h1*p2x+h2*p2y+h3)/(h7*p2x+h8*p2y+h9),
		(h4*p2x+h5*p2y+h6)/(h7*p2x+h8*p2y+h9),
		(h1*p3x+h2*p3y+h3)/(h7*p3x+h8*p3y+h9),
		(h4*p3x+h5*p3y+h6)/(h7*p3x+h8*p3y+h9),
		(h1*p4x+h2*p4y+h3)/(h7*p4x+h8*p4y+h9),
		(h4*p4x+h5*p4y+h6)/(h7*p4x+h8*p4y+h9)
	}
}

// Compute jacobian matrix for g(x) algebraically
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
// outputs: Jg - 9x6 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_g(const float *x, float Jg[][6])
{
	float ox = x[0];
	float oy = x[1];
	float oz = x[2];
	float tx = x[3];
	float ty = x[4];
	float tz = x[5];

	Jg[0] = {-cos(ox)*sin(oy)*sin(oz), 
			 -sin(oy)*cos(oz)-sin(ox)*cos(oy)*sin(oz),
			 -cos(oy)*sin(oz)-sin(ox)*sin(oy)*cos(oz),
			 0, 0, 0};
	Jg[1] = {sin(ox)*sin(oz),
			 0,
			 -cos(ox)*cos(oz),
			 0, 0, 0};
	Jg[2] = {0, 0, 0,
			 1, 0, 0};
	Jg[3] = {cos(ox)*sin(oy)*cos(oz),
			 -sin(oy)*sin(oz)+sin(ox)*cos(oy)*cos(oz),
			 cos(oy)*cos(oz)-sin(ox)*sin(oy)*sin(oz),
			 0, 0, 0};
	Jg[4] = {-sin(ox)*cos(oz),
			 0,
			 -cos(ox)*sin(oz),
			 0, 0, 0};
	Jg[5] = {0, 0, 0,
			 0, 1, 0};
	Jg[6] = {-sin(ox)*sin(oy),
			 cos(ox)*cos(oy),
			 0, 0, 0, 0};
	Jg[7] = {-cos(ox),
			 0, 0, 0, 0, 0};
	Jg[8] = {0, 0, 0,
			 0, 0, -1};

}

// Compute jacobian matrix for f(h) algebraically
// inputs: h - 1x9 array of homography values
// 			   [h1 h2 h3 h4 h5 h6 h7 h8 h9]
// outputs: Jf - 8x9 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_f(const float *h, float Jf[][9])
{

}

// Computes the step in which to move our current estimate on x.
// Returns false if matrix inverse operation could not be succesfully completed.
//
// inputs: J - 8x6 2D array of joint jacobian
//		   f - 1x8 array of estimated x,y coordinates of all 4 photodiodes
//			   [x1 y1 x2 y2 ... x4 y4]
// 		   projection2D - 1x8 array of measured x,y coordinates of all 4 photodiodes
//						  [x1 y1 x2 y2 ... x4 y4]
//
// outputs: deltaX - 1x6 array of the step in which to move x
//					 [deltaThetaX deltaThetaY deltaThetaZ deltaX deltaY deltaZ]
bool LevenbergMarquardtPose::compute_delta_x(const float* J, const float* f, const float* projection2D, float* deltaX) {
	int inv = 0;
	return true;
}


// Estimates 3D pose of VRduino via Levengerg-Marquardt Algorithm.
// inputs : projection2D - 1x8 array of the x,y projection of each diode
//                         onto the light house "sensor" plane  
//			poseEstimate  -	1x6 array specifying the initial guess of the 3D
//							   	pose of the VRduino (could be random, output
//							   	from homography method, or estimate from last
// 							   	frame)
// output:  pose3D       - member variable storing VRduino 3D pose
// 						   [thetax, thetay, thetaz,x,y,z]
bool LevenbergMarquardtPose::computePosition(const float* projection2D,
        const float* poseEstimate) {


	bool successLM = true;

	// memcpy(pose3D, ..., 6*sizeof(float));

	return successLM;
}

// Compute L2 error between estimated x,y diode positions on light house
// "sensor" and the measured ones.
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
//  	   projection2D - 1x8 array of the x,y projection of each diode
//                        onto the light house "sensor" plane  
// outputs : scalar L2 error
float LevenbergMarquardtPose::get_residual(const float *x, const float *projection2D)
{
	//float h[12];
	float h[9] = {};
	eval_g_at_x(x, h);
	float f[8] = {};
	eval_f_at_h(h, f);

	return (projection2D[0] - f[0]) * (projection2D[0] - f[0]) + (projection2D[1] - f[1]) * (projection2D[1] - f[1]) + (projection2D[2] - f[2]) * (projection2D[2] - f[2]) + (projection2D[3] - f[3]) * (projection2D[3] - f[3]) + (projection2D[4] - f[4]) * (projection2D[4] - f[4]) + (projection2D[5] - f[5]) * (projection2D[5] - f[5]) + (projection2D[6] - f[6]) * (projection2D[6] - f[6]) + (projection2D[7] - f[7]) * (projection2D[7] - f[7]);
}

// Compute jacobian matrix for g(x) numerically. This is purely for debug
// purposes, as a reference to check if the algebraic jacobian is correctly
// implemented.
// inputs: x - 1x6 array of current estimate on rotation/translation parameters
// 			   [thetaX thetaY thetaZ X Y Z]
// outputs: Jg - 9x6 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_g_numeric(const float *x, float Jg[][6])
{

	float deltaX = 0.01;

	float h_x[9];
	eval_g_at_x(x, h_x);

	for (int i = 0; i < 6; i++) {

		float xx[6];
		memcpy(xx, x, 6 * sizeof(float));
		xx[i] += deltaX;

		float h_xx[9];
		eval_g_at_x(xx, h_xx);

		for (int j = 0; j < 9; j++)
		{
			Jg[j][i] = (h_xx[j] - h_x[j]) / deltaX;
		}
	}
}

// Compute jacobian matrix for f(h) numerically. This is purely for debug
// purposes, as a reference to check if the algebraic jacobian is correctly
// implemented.
// inputs: h - 1x9 array of homography values
// 			   [h1 h2 h3 h4 h5 h6 h7 h8 h9]
// outputs: Jg - 9x6 2D array jacobian of g(x)
void LevenbergMarquardtPose::get_jacobian_f_numeric(const float *h, float Jf[][9])
{

	float deltaX = 0.000001;

	float f_x[8];
	eval_f_at_h(h, f_x);

	for (int i = 0; i < 9; i++) {

		float hh[9];
		memcpy(hh, h, 9 * sizeof(float));
		hh[i] += deltaX;

		float f_xx[8];
		eval_f_at_h(hh, f_xx);

		for (int j = 0; j < 8; j++)
		{
			Jf[j][i] = (f_xx[j] - f_x[j]) / deltaX;
		}
	}
}

// Debug function to compare algebraic jacobians with numeric
void LevenbergMarquardtPose::check_jacobians(void)
{
	// Testing get_jacobian_g()
	float testx[6];
	for (int j = 0; j < 6; j++) {
		testx[j] = float(random(999999)) / 999999.0;
	}
	float Jg1[9][6];
	float Jg2[9][6];
	get_jacobian_g(testx, Jg1);
	get_jacobian_g_numeric(testx, Jg2);
	float errorJg = Utils::ComputeL2Error((float*)Jg1, (float*)Jg2, 9,6);
	Serial.printf("Error Jg: %.9g \n", errorJg);

	// Testing get_jacobian_f()
	float testh[9];
	for (int j = 0; j < 9; j++) {
		testh[j] = (float(random(999999)) / 999999.0);
	}
	float Jf1[8][9];
	float Jf2[8][9];
	get_jacobian_f(testh, Jf1);	
	get_jacobian_f_numeric(testh, Jf2);
	float errorJf = Utils::ComputeL2Error((float*)Jf1, (float*)Jf2, 8,9);
	Serial.printf("Error Jf: %.9g \n", errorJf);

}