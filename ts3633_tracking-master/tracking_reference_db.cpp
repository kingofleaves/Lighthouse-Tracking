#include "tracking_reference_db.h"
#include <Arduino.h>

tracking_reference_db::tracking_reference_db(void) {
  num_references = 0;
  reference_list = NULL;
}

void tracking_reference_db::add_tracking_reference(uint32_t basestation_id, float x, float y, float z, float r_w, float r_x, float r_y, float r_z) {
  new_reference = (t_ref_ll_t *)malloc(sizeof(t_ref_ll_t));
  new_reference->obj.basestation_id = basestation_id;
  new_reference->obj.pose.pos.x = fixed(x,28);
  new_reference->obj.pose.pos.y = fixed(y,28);
  new_reference->obj.pose.pos.z = fixed(z,28);
  new_reference->obj.pose.rotation.w = fixed(r_w,28);
  new_reference->obj.pose.rotation.x = fixed(r_x,28);
  new_reference->obj.pose.rotation.y = fixed(r_y,28);
  new_reference->obj.pose.rotation.z = fixed(r_z,28);
  new_reference->next = reference_list;
  reference_list = new_reference;
  num_references++;
}

reference_pose_rtn_t tracking_reference_db::get_reference_pose(uint32_t basestation_id) {
  reference_pose_rtn_t rtn;
  rtn.valid = false;
  t_ref_ll_t *kth_ref;
  for (kth_ref = reference_list; kth_ref != NULL; kth_ref = kth_ref->next) {
    if (kth_ref->obj.basestation_id == basestation_id) {
      rtn.valid = true;
      rtn.pose = kth_ref->obj.pose;
    }
  }
  return rtn;
}


void tracking_reference_db::print_tracking_reference(){
  t_ref_ll_t *kth_ref;
  for (kth_ref = reference_list; kth_ref != NULL; kth_ref = kth_ref->next) {
    Serial.println("--------------------------------");
    Serial.println(kth_ref->obj.basestation_id,HEX);
    Serial.print((float) kth_ref->obj.pose.pos.x);
    Serial.print(", ");
    Serial.print((float) kth_ref->obj.pose.pos.y);
    Serial.print(", ");
    Serial.println((float) kth_ref->obj.pose.pos.z);
    Serial.print((float) kth_ref->obj.pose.rotation.w);
    Serial.print(", ");
    Serial.print((float) kth_ref->obj.pose.rotation.x);
    Serial.print(", ");
    Serial.print((float) kth_ref->obj.pose.rotation.y);
    Serial.print(", ");
    Serial.println((float) kth_ref->obj.pose.rotation.z);
  }
}

