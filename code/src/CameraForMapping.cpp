#include "CameraForMapping.h"
#include "BundleAdj.h"
#include "PointsContainer.h"

void CameraForMapping::setGrountruthPose(){
  assignPose(*(grountruth_camera_->frame_camera_wrt_world_));
}

void CameraForMapping::setPoseToWorldReferenceFrame(){
  Eigen::Isometry3f T_world;
  T_world.linear().setIdentity();
  T_world.translation().setZero();
  assignPose(T_world);
}

PointsContainer* CameraForMapping::initializePointsContainer(){
  PointsContainer* ptr = new PointsContainer(this);
  return ptr;
}
CamDataForBA* CameraForMapping::initializeDataForBA(){
  CamDataForBA* ptr = new CamDataForBA();
  return ptr;
}
