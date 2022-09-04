#include "CameraForMapping.h"
#include "BundleAdj.h"
#include "PointsContainer.h"

void CameraForMapping::cam_free_mem(){

  set_free_=true;

  for( Candidate* cand : points_container_->candidates_ )
    delete cand;
  points_container_->candidates_.clear();

  points_container_->clearProjections();

  if (pyramid_!=nullptr){
     delete pyramid_;
     pyramid_=nullptr;
  }
  // if (cam_data_for_ba_!=nullptr){
  //    delete cam_data_for_ba_;
  //    cam_data_for_ba_=nullptr;
  // }
  if(image_intensity_!=nullptr){
     delete image_intensity_;
     image_intensity_=nullptr;
  }
  if(invdepth_map_!=nullptr){
     delete invdepth_map_;
     invdepth_map_=nullptr;
   }

}

void CameraForMapping::destructor(){
  if (!set_free_){
    if (pyramid_!=nullptr){
       delete pyramid_;
       pyramid_=nullptr;
    }
    if (cam_data_for_ba_!=nullptr){
       delete cam_data_for_ba_;
       cam_data_for_ba_=nullptr;
    }
    if(image_intensity_!=nullptr){
       delete image_intensity_;
       image_intensity_=nullptr;
    }
    if(invdepth_map_!=nullptr){
       delete invdepth_map_;
       invdepth_map_=nullptr;
     }
  }

  // // delete points container;
  // if(points_container_!=nullptr){
  //   delete points_container_;
  //    points_container_=nullptr;
  //  }

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
