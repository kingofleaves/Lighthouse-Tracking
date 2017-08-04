#ifndef _Utils_h_
#define _Utils_h_

namespace Utils {

inline void print2DArray(const float* array, const int x, const int y) {
	for (int i = 0; i < x; ++i) {
		for (int j = 0; j < y; ++j) {
			Serial.printf("%f ", array[i * y + j]);
		}
		Serial.println();
	}
}

inline void print2DArray(const int* array, const int x, const int y) {
	for (int i = 0; i < x; ++i) {
		for (int j = 0; j < y; ++j) {
			Serial.printf("%d ", array[i * y + j]);
		}
		Serial.println();
	}
}

inline float ComputeL2Error(const float* A, const float* B, const int x, const int y) {
	float error = 0.0;
	for (int i = 0; i < x; ++i) {
		for (int j = 0; j < y; ++j) {
			error += sq(A[i*y+j] - B[i*y+j]);
		}
	}
	return error;
}

}
#endif