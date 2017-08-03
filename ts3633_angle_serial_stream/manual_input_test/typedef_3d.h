#ifndef typedef_3d_h
#define typedef_3d_h

#include "third_party_includes/Fp32s.hpp"
#ifndef fixed
  #define fixed Fp::Fp32s
#endif

// Create Structures for Vector math objects
typedef struct quaternion_t {
  fixed w;
  fixed x;
  fixed y;
  fixed z;
} quaternion_t;

typedef struct vec3_t{
  fixed x;
  fixed y;
  fixed z;
} vec3_t;

typedef struct float_vec3_t{
  float x;
  float y;
  float z;
} float_vec3_t;

typedef struct {
  vec3_t pos;
  quaternion_t rotation;
} coord_t;

#endif
