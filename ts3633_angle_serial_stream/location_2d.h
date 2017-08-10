#include <stdint.h>

#ifndef location_2d_h
#define location_2d_h

#define SENSOR_1_HORIZONTAL 0
#define SENSOR_1_VERTICAL 1
#define SENSOR_2_HORIZONTAL 2
#define SENSOR_2_VERTICAL 3

#define SCALER_EPS 0.00001

typedef union vec2_t{
  struct {
    float x;
    float y;
  };
  struct {
    float horizontal;
    float vertical;
  };
} vec2_t;

typedef struct location_t{
  vec2_t position;
  float orientation;
  float distance_to_plane;
} location_t;

// Define the class for locating 2d position of Zooids
class LOC2D {
	public:
		LOC2D(void);
    bool init(float normal_horizontal, float normal_vertical, float x_scaler, float y_scaler);
    location_t get_location(float *angles); // angles should be an array of size 4: H1, V1, H2, V2

    bool debug = false;
    bool variable_distance = false; // if true, calculates 2d position based on the separation of the two sensors 
    float sensor_separation = 0.030; // in meters

	private:
    // TODO: Work in Progress
    vec2_t angle_to_plane_normal;
    vec2_t scaler;
};
#endif
