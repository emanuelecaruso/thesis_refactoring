#pragma once
#include "camera.h"
#include "Pyramid.h"
// #include "PointsContainer.h"

class PointsContainer;

class CameraForMapping : public Camera, public std::enable_shared_from_this<CameraForMapping>{
  // class CameraForMapping : public Camera{
  public:
    // ********** members **********
    std::shared_ptr<Camera> grountruth_camera_;
    std::shared_ptr<Pyramid> pyramid_;
    // Pyramid pyramid_;
    std::shared_ptr<PointsContainer> points_container_;

    // flags
    bool fixed_;
    bool keyframe_;

    // ********** constructor **********

    CameraForMapping(std::shared_ptr<Camera> env_cam, std::shared_ptr<Params> parameters):
      Camera(env_cam)
      ,grountruth_camera_(env_cam)
      ,pyramid_( new Pyramid(env_cam,parameters->coarsest_level) )
      // ,points_container_( new PointsContainer(this) )
      ,points_container_( initializePointsContainer(parameters) )
      { };


    // ********** methods **********
    void setGrountruthPose();
  protected:
    std::shared_ptr<PointsContainer> initializePointsContainer(std::shared_ptr<Params> parameters);


};
