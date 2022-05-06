#include "CameraForMapping.h"
#include "BundleAdj.h"
#include "PointsContainer.h"

void CameraForMapping::setGrountruthPose(){
  assignPose(*(grountruth_camera_->frame_camera_wrt_world_));
}

PointsContainer* CameraForMapping::initializePointsContainer(){
  PointsContainer* ptr = new PointsContainer(this);
  return ptr;
}
CamDataForBA* CameraForMapping::initializeDataForBA(){
  CamDataForBA* ptr = new CamDataForBA();
  return ptr;
}
