#pragma once
#include "camera.h"
#include "CameraForMapping.h"

// class CameraForMapping;

class Candidate{
  public:

  // ********** members **********
  std::shared_ptr<CameraForMapping> cam_;
  int level_;
  pxl pixel_;
  Eigen::Vector2f uv_;

  // ********** constructor **********
  // construct candidate from pixel
  Candidate(std::shared_ptr<CameraForMapping> cam, pxl& pixel, int level):
  level_(level)
  ,cam_(cam)
  ,pixel_(pixel){
    cam->pixelCoords2uv(pixel, uv_, level);
  }
  // ********** methods **********

};

class ActivePoint{
public:

  // ********** members **********
  std::shared_ptr<CameraForMapping> cam_;
  int level_;
  pxl pixel_;
  Eigen::Vector2f uv_;

  // current guess


  // ********** constructor **********

  // ********** methods **********

};


class PointsContainer{
  public:
    // ********** members **********
    std::shared_ptr<Params> parameters_;
    std::shared_ptr<CameraForMapping> cam_;
    std::vector<std::shared_ptr<Candidate>> candidates_;
    std::vector<std::shared_ptr<ActivePoint>> active_points_;

    // ********** constructor **********
    PointsContainer(CameraForMapping* cam, std::shared_ptr<Params> parameters):
    parameters_(parameters)
    ,cam_(cam)
    {};


    // ********** methods **********

  protected:
};
