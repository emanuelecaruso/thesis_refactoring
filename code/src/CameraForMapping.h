#pragma once
#include "camera.h"
// #include "BundleAdj.h"
#include "Pyramid.h"
// #include "PointsContainer.h"

class PointsContainer;
class CamDataForBA;

class PoseNormError{
  public:
    float angle;
    float position_norm;

    // default constructor
    PoseNormError():
    angle(0),
    position_norm(0)
    {  }

    // assignment constructor
    PoseNormError(float angle_, float position_error_norm):
    angle(angle_),
    position_norm(position_error_norm)
    {  }

    // += operator
    PoseNormError& operator+=(const PoseNormError& rhs){
      /* addition of rhs to *this */
      assert(!std::isnan(rhs.angle));
      assert(!std::isnan(rhs.position_norm));
      this->angle += rhs.angle;
      this->position_norm += rhs.position_norm;

      return *this; // return the result by reference
    }

    // /= operator
    PoseNormError& operator/=(float divisor){
      /* addition of rhs to *this */
      assert(divisor>0);
      this->angle /= divisor;
      this->position_norm /= divisor;

      return *this; // return the result by reference
    }

    inline void print(){
      std::cout << "angle error: " << angle << std::endl;
      std::cout << "position error norm: " << position_norm << std::endl;
    }

};


class CameraForMapping : public Camera{
  // class CameraForMapping : public Camera{
  public:
    // ********** members **********
    Camera* grountruth_camera_;
    Pyramid* pyramid_;
    // Pyramid pyramid_;
    PointsContainer* points_container_;
    CamDataForBA* cam_data_for_ba_;

    // flags
    bool set_free_;
    bool fixed_;
    bool keyframe_;
    bool marginalized_;
    bool has_active_pts_;
    bool discarded_during_initialization;

    // ********** constructor **********

    CameraForMapping(Camera* env_cam):
      Camera(env_cam,false)
      ,grountruth_camera_(env_cam)
      ,pyramid_( new Pyramid(env_cam,coarsest_level_pyr) )
      ,cam_data_for_ba_( initializeDataForBA() )
      // ,points_container_( new PointsContainer(this) )
      ,points_container_( initializePointsContainer() )
      ,fixed_(false)
      ,keyframe_(false)
      ,marginalized_(false)
      ,has_active_pts_(false)
      ,discarded_during_initialization(false)
      ,set_free_(false)
      { };

    ~CameraForMapping(){
      destructor();
    }

    // ********** methods **********
    void setGrountruthPose();
    void setPoseToWorldReferenceFrame();
    void cam_free_mem();

    inline PoseNormError getPoseNormError(){
      // get relative transformation matrix wrt groundtruth
      Eigen::Isometry3f relative_transf = (*(grountruth_camera_->frame_world_wrt_camera_))*(*(frame_camera_wrt_world_));
      assert(frame_camera_wrt_world_->linear().allFinite());
      assert(grountruth_camera_->frame_world_wrt_camera_->linear().allFinite());
      assert(frame_camera_wrt_world_->translation().allFinite());
      assert(grountruth_camera_->frame_world_wrt_camera_->translation().allFinite());

      // get angle from rotation matrix
      float angle = rotation2angle(relative_transf.linear());

      // get norm of translation vector
      float norm_transl = relative_transf.translation().norm();

      PoseNormError pose_norm_error(angle,norm_transl);
      return pose_norm_error;
    };

  protected:
    PointsContainer* initializePointsContainer();
    CamDataForBA* initializeDataForBA();
    void destructor();

};
