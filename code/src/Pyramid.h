#pragma once
#include "image.h"
#include "camera.h"
#include <memory>

class CameraForMapping;
class Pyramid;

class PyramidLevel{
  public:
    // ********** members **********
    std::shared_ptr<Image<pixelIntensity>> c_;
    std::shared_ptr<Image<pixelIntensity>> c_dx_;
    std::shared_ptr<Image<pixelIntensity>> c_dy_;
    std::shared_ptr<Image<float>> magn_cd_;
    std::shared_ptr<Image<float>> magn_cd_dx_;
    std::shared_ptr<Image<float>> magn_cd_dy_;
    std::shared_ptr<Image<float>> phase_cd_;

    // ********** constructor **********
    PyramidLevel(std::shared_ptr<Image<pixelIntensity>> img){
      init(img);
    }
    ~PyramidLevel(){}

  protected:
    // ********** methods **********
    void init(std::shared_ptr<Image<pixelIntensity>> img);
    std::shared_ptr<Image<float>> getMagnitude(std::shared_ptr<Image<pixelIntensity>> dx, std::shared_ptr<Image<pixelIntensity>> dy);
    std::shared_ptr<Image<float>> getPhase(std::shared_ptr<Image<pixelIntensity>> dx, std::shared_ptr<Image<pixelIntensity>> dy);
};

class Pyramid{
  public:
    // ********** members **********
    int levels_;
    std::vector< std::shared_ptr<PyramidLevel> > pyramid_levels_;

    // ********** constructor **********
    Pyramid(){}
    Pyramid(std::shared_ptr<Camera> cam, const int levels):
    levels_(levels){
      init(levels, cam->image_intensity_);
    }

    ~Pyramid(){}

    // ********** methods **********
    void init(const int levels, std::shared_ptr<Image<pixelIntensity>> image_intensity);

  protected:

};
