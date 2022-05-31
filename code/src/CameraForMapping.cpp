#include "CameraForMapping.h"
#include "BundleAdj.h"
#include "PointsContainer.h"

void CameraForMapping::cam_free_mem(){
  delete pyramid_;
  points_container_->candidates_.clear();
  points_container_->candidates_projected_.clear();
  points_container_->active_points_projected_.clear();
  points_container_->marginalized_points_projected_.clear();
  delete cam_data_for_ba_;
  if(image_intensity_!=nullptr)
    delete image_intensity_;
  if(invdepth_map_!=nullptr)
    delete invdepth_map_;
}

void CameraForMapping::destructor(){
  delete pyramid_;
  delete grountruth_camera_;
  delete points_container_;
  delete cam_data_for_ba_;
}

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
