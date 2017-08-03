#include "typedef_3d.h"
#include <stdint.h>

#ifndef tracking_reference_db_h
#define tracking_reference_db_h

typedef struct tracking_reference_t {
  uint32_t basestation_id;
  coord_t pose;
} tracking_reference_t;

typedef struct reference_pose_rtn_t {
  bool valid;
  coord_t pose;
} reference_pose_rtn_t;

// Create a structure for a linked list
typedef struct _t_ref_ll_t
{
    tracking_reference_t obj;
    struct _t_ref_ll_t *next;
} t_ref_ll_t;

// TODO: Try to get these references inside the scope of the tracking_reference_db class,
// Not really sure why the compiler doesn't like it when I add them under the private members of the class
static t_ref_ll_t *reference_list, *new_reference;

class tracking_reference_db {
  public:
    tracking_reference_db(void);
    void add_tracking_reference(uint32_t basestation_id, float x, float y, float z, float r_w, float r_x, float r_y, float r_z);
    reference_pose_rtn_t get_reference_pose(uint32_t basestation_id);
    void print_tracking_reference();
  private:
    uint16_t num_references;
};

#endif