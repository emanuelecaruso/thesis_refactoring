#pragma once
#include "camera.h"
#include "Pyramid.h"

class CameraForMapping : public Camera{
  // class CameraForMapping : public Camera{
  public:
    // ********** members **********
    std::shared_ptr<Camera> grountruth_camera_;
    std::shared_ptr<Pyramid> pyramid_;

    // flags
    bool fixed_;
    bool keyframe_;

    // ********** constructor **********

    CameraForMapping(std::shared_ptr<Camera> env_cam, std::shared_ptr<Params> parameters):
      Camera(env_cam)
      ,grountruth_camera_(env_cam)
      ,pyramid_( new Pyramid(env_cam,parameters->coarsest_level) )
      { };


    // ********** methods **********
    void setGrountruthPose();
};
