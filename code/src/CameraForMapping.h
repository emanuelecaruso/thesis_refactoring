#pragma once
#include "camera.h"
#include "Pyramid.h"
// #include "PointsContainer.h"

class PointsContainer;

class CameraForMapping : public Camera, public std::enable_shared_from_this<CameraForMapping>{
  // class CameraForMapping : public Camera{
  public:
    // ********** members **********
    Camera* grountruth_camera_;
    Pyramid* pyramid_;
    // Pyramid pyramid_;
    PointsContainer* points_container_;

    // flags
    bool fixed_;
    bool keyframe_;

    // ********** constructor **********

    CameraForMapping(Camera* env_cam, Params* parameters):
      Camera(env_cam,false)
      ,grountruth_camera_(env_cam)
      ,pyramid_( new Pyramid(env_cam,parameters->coarsest_level) )
      // ,points_container_( new PointsContainer(this) )
      ,points_container_( initializePointsContainer(parameters) )
      ,fixed_(false)
      ,keyframe_(false)
      { };


    // ********** methods **********
    void setGrountruthPose();
  protected:
    PointsContainer* initializePointsContainer(Params* parameters);


};
