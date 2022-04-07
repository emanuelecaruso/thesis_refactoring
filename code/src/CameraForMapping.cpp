#include "CameraForMapping.h"

void CameraForMapping::setGrountruthPose(){
  assignPose(*(grountruth_camera_->frame_camera_wrt_world_));
}
