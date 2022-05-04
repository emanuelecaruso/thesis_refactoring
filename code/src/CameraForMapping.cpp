#include "CameraForMapping.h"
#include "BundleAdj.h"
#include "PointsContainer.h"

void CameraForMapping::setGrountruthPose(){
  assignPose(*(grountruth_camera_->frame_camera_wrt_world_));
}

PointsContainer* CameraForMapping::initializePointsContainer(Params* parameters){
  PointsContainer* ptr = new PointsContainer(this, parameters);
  return ptr;
}
DataForBA* CameraForMapping::initializeDataForBA(){
  DataForBA* ptr = new DataForBA();
  return ptr;
}
