#pragma once
#include "camera.h"
// #include "BundleAdj.h"
#include "Pyramid.h"
// #include "PointsContainer.h"

class PointsContainer;
class CamDataForBA;

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
    bool fixed_;
    bool keyframe_;
    bool marginalized_;

    // ********** constructor **********

    CameraForMapping(Camera* env_cam):
      Camera(env_cam,false)
      ,grountruth_camera_(env_cam)
      ,pyramid_( new Pyramid(env_cam,coarsest_level) )
      ,cam_data_for_ba_( initializeDataForBA() )
      // ,points_container_( new PointsContainer(this) )
      ,points_container_( initializePointsContainer() )
      ,fixed_(false)
      ,keyframe_(false)
      ,marginalized_(false)
      { };

    ~CameraForMapping(){
      delete pyramid_;
    }

    // ********** methods **********
    void setGrountruthPose();
  protected:
    PointsContainer* initializePointsContainer();
    CamDataForBA* initializeDataForBA();


};
