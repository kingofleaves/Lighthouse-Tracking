#include "location_2d.h"
#include <Arduino.h>

// Constructor, initialize variables just like we are waiting for the sync word
LOC2D::LOC2D(void) {
  angle_to_plane_normal.horizontal = 0;
  angle_to_plane_normal.vertical = 0;
  scaler.x = 0;
  scaler.y = 0;
}

bool LOC2D::init(float normal_horizontal, float normal_vertical, float x_scaler, float y_scaler) {
  angle_to_plane_normal.horizontal = normal_horizontal;
  // normalize to range of -180 to 180
  while (angle_to_plane_normal.horizontal > 180) {
    angle_to_plane_normal.horizontal - 360;
  }
  while (angle_to_plane_normal.horizontal < -180) {
    angle_to_plane_normal.horizontal + 360;
  }

  angle_to_plane_normal.vertical = normal_vertical;
  // normalize to range of -180 to 180
  while (angle_to_plane_normal.vertical > 180) {
    angle_to_plane_normal.vertical - 360;
  }
  while (angle_to_plane_normal.vertical < -180) {
    angle_to_plane_normal.vertical + 360;
  }

  scaler.x = x_scaler;
  scaler.y = y_scaler;
  // check for scaler = 0
  if (scaler.x < SCALER_EPS || scaler.y < SCALER_EPS) {   // in meters
    return false;
  }
  return true;
}


location_t LOC2D::get_location(float *angles) {
  vec2_t sensor_1_pos;
  vec2_t sensor_2_pos;  

  sensor_1_pos.x = tan((angles[SENSOR_1_HORIZONTAL] - angle_to_plane_normal.horizontal) * 3.14159/180.0);
  sensor_1_pos.y = tan((angles[SENSOR_1_VERTICAL] - angle_to_plane_normal.vertical) * 3.14159/180.0);
  sensor_2_pos.x = tan((angles[SENSOR_2_HORIZONTAL] - angle_to_plane_normal.horizontal) * 3.14159/180.0);
  sensor_2_pos.y = tan((angles[SENSOR_2_VERTICAL] - angle_to_plane_normal.vertical) * 3.14159/180.0);

  if (variable_distance) {
    float distance = sensor_separation / sqrt(pow(sensor_1_pos.x - sensor_2_pos.x,2) + pow(sensor_1_pos.y - sensor_2_pos.y,2));
    sensor_1_pos.x *= distance;
    sensor_1_pos.y *= distance;
    sensor_2_pos.x *= distance;
    sensor_2_pos.y *= distance;
  }

  if (debug) {
    Serial.print("sensor 1: X: ");
    Serial.println(sensor_1_pos.x);
    Serial.print("sensor 1: Y: ");
    Serial.println(sensor_1_pos.y);
    Serial.print("sensor 2: X: ");
    Serial.println(sensor_2_pos.x);
    Serial.print("sensor 2: Y: ");
    Serial.println(sensor_2_pos.y);
    Serial.print("Separation: ");
    Serial.println(sqrt(pow(sensor_1_pos.x - sensor_2_pos.x,2) + pow(sensor_1_pos.y - sensor_2_pos.y,2)));
  }
  // TODO: Check calculations for 2D

  location_t location;
  location.position.x = scaler.x * (sensor_1_pos.x + sensor_2_pos.x)/2;
  location.position.y = scaler.y * (sensor_1_pos.y + sensor_2_pos.y)/2;
  location.orientation = atan2(sensor_1_pos.y - sensor_2_pos.y, sensor_1_pos.x - sensor_2_pos.x) * 180/3.14159;

  return location;
}
