#include "CameraForMapping.h"
#include "PointsContainer.h"

void CameraForMapping::setGrountruthPose(){
  assignPose(*(grountruth_camera_->frame_camera_wrt_world_));
}

std::shared_ptr<PointsContainer> CameraForMapping::initializePointsContainer(std::shared_ptr<Params> parameters){
  std::shared_ptr<PointsContainer> ptr(new PointsContainer(this, parameters));
  return ptr;
}
