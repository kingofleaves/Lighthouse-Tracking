#include "LevenbergMarquardt.h"
#include "WProgram.h"
#include "Utils.h"

void LevenbergMarquardtPose::unit_tests() {
	test_eval_g_at_x();
	test_eval_f_at_h();
	test_get_jacobian_f();
	test_get_jacobian_g();
	test_compute_delta_x();
	test_computePosition();
}

void LevenbergMarquardtPose::test_eval_g_at_x() {
	float testx[6] = {-0.258478045, 0.180611506, -0.144890532, -52.9364243, -994.535278, -1492.2207};
	float correcth[9] = {0.966796696, 0.1395877, -52.9364243, -0.18746987, 0.956649959, -994.535278, 0.173663855, 0.255609453, 1492.2207};
	float h[9] = {};

	eval_g_at_x(testx, h);

	Serial.println("Testing LevenbergMarquardtPose::eval_g_at_x()");
  	float diff = Utils::ComputeL2Error((float*)correcth, (float*)h, 1,9);
  	Serial.printf("Difference in h: %.5g \n", diff);
}

void LevenbergMarquardtPose::test_eval_f_at_h() {
	float testh[9] = {0.966796696, 0.1395877, -52.9364243, -0.18746987, 0.956649959, -994.535278, 0.173663855, 0.255609453, 1492.2207};
	float correctf[8] = {-0.0603843406, -0.645567179, -0.00587106869, -0.649770677, -0.0105956718, -0.687367618, -0.0656267703, -0.683498681};
	float f[8] = {};

	eval_f_at_h(testh, f);

	Serial.println("Testing LevenbergMarquardtPose::eval_f_at_h()");
  	float diff = Utils::ComputeL2Error((float*)correctf, (float*)f, 1,8);
  	Serial.printf("Difference in f: %.5g \n", diff);
}

void LevenbergMarquardtPose::test_get_jacobian_f() {
	float testh[9] = {0.966796696, 0.1395877, -52.9364243, -0.18746987, 0.956649959, -994.535278, 0.173663855, 0.255609453, 1492.2207};
	float correctJf[8][9] = {
		{-0.0281630252, 0.0167637058, 0.000670548237, 0, 0, 0, -0.00170060573, 0.00101226533, 4.04906132e-05},
		{0, 0, 0, -0.0281630252, 0.0167637058, 0.000670548237, -0.0181811247, 0.0108220978, 0.000432883913},
		{0.0278902091, 0.0166013148, 0.000664052612, 0, 0, 0, 0.000163745339, 9.74674695e-05, 3.89869865e-06},
		{0, 0, 0, 0.0278902091, 0.0166013148, 0.000664052612, 0.018122239, 0.0107870474, 0.000431481894},
		{0.028128935, -0.0167434141, 0.00066973659, 0, 0, 0, 0.000298044964, -0.000177407725, 7.09630876e-06},
		{0, 0, 0, 0.028128935, -0.0167434141, 0.00066973659, 0.0193349179, -0.0115088802, 0.000460355193},
		{-0.0284064654, -0.0169086102, 0.00067634444, 0, 0, 0, -0.00186422456, -0.00110965746, 4.43862991e-05},
		{0, 0, 0, -0.0284064654, -0.0169086102, 0.00067634444, -0.0194157828, -0.0115570128, 0.000462280528}
	};
	float Jf[8][9] = {};

	get_jacobian_f(testh, Jf);

	Serial.println("Testing LevenbergMarquardtPose::get_jacobian_f()");
  	float diff = Utils::ComputeL2Error((float*)correctJf, (float*)Jf, 8,9);
  	Serial.printf("Difference in Jf: %.5g \n", diff);
}

void LevenbergMarquardtPose::test_get_jacobian_g() {
	float testx[6] = {-0.258478045, 0.180611506, -0.144890532, -52.9364243, -994.535278, -1492.2207};
	float correctJg[9][6] = {
		{0.0250743013, -0.214054585, 0.18746987, 0, 0, 0, },
		{0.0369059443, 0, -0.956649959, 0, 0, 0, },	
		{0, 0, 0, 1, 0, 0, },
		{0.171844155, -0.222881034, 0.966796696, 0, 0, 0, },
		{0.252931118, 0, 0.1395877, 0, 0, 0, },
		{0, 0, 0, 0, 1, 0, },
		{0.0459154248, 0.951054513, 0, 0, 0, 0, },
		{-0.966780126, 0, 0, 0, 0, 0, },
		{0, 0, 0, 0, 0, -1, },
	};
	float Jg[9][6] = {};

	get_jacobian_g(testx, Jg);

	Serial.println("Testing LevenbergMarquardtPose::get_jacobian_g()");
  	float diff = Utils::ComputeL2Error((float*)correctJg, (float*)Jg, 9,6);
  	Serial.printf("Difference in Jg: %.5g \n", diff);
}

void LevenbergMarquardtPose::test_compute_delta_x() {
	float testJ[8][6] = {
		{-0.00114420976, 0.00441105571, -0.0213167164, 0.000670548237, 0, -4.04906132e-05},
		{-0.0118969707, -0.0110142361, -0.024887912, 0, 0.000670548237, -0.000432883913},
		{0.00122530351, -0.00581429666, -0.010653073, 0.000664052612, 0, -3.89869865e-06},	
		{-0.000604854897, 0.0110190399, 0.0292815007, 0, 0.000664052612, -0.000431481894},
		{0.000272581034, -0.0057376707, 0.0212909132, 0.00066973659, 0, -7.09630876e-06},
		{0.0126131903, 0.0121191535, 0.0248577874, 0, 0.00066973659, -0.000460355193},
		{-0.000349102425, 0.00430755503, 0.0108502656, 0.00067634444, 0, -4.43862991e-05},
		{0.00112340692, -0.0121342065, -0.0298235118, 0, 0.00067634444, -0.000462280528},
	};
	float testf[8] = {-0.0603843406, -0.645567179, -0.00587106869, -0.649770677, -0.0105956718, -0.687367618, -0.0656267703, -0.683498681};
	float testProj2D[8] = {-0.0586576685, -0.645769835, -0.00732780714, -0.649426341, -0.0126140844, -0.68708843, -0.0638766289, -0.684098065};
	float correctDeltaX[6] = {-0.0530769601, 0.142360643, -0.0306434017, 0.132885739, -0.298111141, -0.42290014};
	float deltaX[6] = {};

	compute_delta_x((float*)testJ, testf, testProj2D, deltaX);

	Serial.println("Testing LevenbergMarquardtPose::compute_delta_x()");
  	float diff = Utils::ComputeL2Error((float*)correctDeltaX, (float*)deltaX, 1,6);
  	Serial.printf("Difference in deltaX: %.5g \n", diff);

}

// This is only valid when the number of LM iterations is set to 3!!!
// If you get any errors from this function make sure you check that parameter!
void LevenbergMarquardtPose::test_computePosition() {
	float testProj2D[8] = {-0.0586576685, -0.645769835, -0.00732780714, -0.649426341, -0.0126140844, -0.68708843, -0.0638766289, -0.684098065};
	float testPosGuess[6] = {0.0, 0.0, 0.0, -52.753437, -992.644104, -1490.77539};
	float correctPose3D[6] = {-0.311554998, 0.322972149, -0.175533935, -52.8035393, -994.833374, -1492.64355};

	computePosition(testProj2D, testPosGuess);

	Serial.println("Testing LevenbergMarquardtPose::computePosition()");
  	float diff = Utils::ComputeL2Error((float*)correctPose3D, (float*)pose3D, 1,3);
  	Serial.printf("Difference in position3D: %.5g \n", diff);

}