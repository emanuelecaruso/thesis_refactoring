#pragma once
#include "image.h"
#include "camera.h"
#include <memory>

class CameraForMapping;
class Pyramid;

class PyramidLevel{
  public:
    // ********** members **********
    Image<pixelIntensity>* c_;
    Image<pixelIntensity>* c_dx_;
    Image<pixelIntensity>* c_dy_;
    Image<pixelIntensity>* magn_cd_;
    Image<pixelIntensity>* magn_cd_dx_;
    Image<pixelIntensity>* magn_cd_dy_;
    Image<pixelIntensity>* magn_cdd_;
    Image<pixelIntensity>* phase_cd_;

    // ********** constructor **********
    PyramidLevel(Image<pixelIntensity>* img){
      init(img);
    }
    ~PyramidLevel(){
      delete c_;
      delete c_dx_;
      delete c_dy_;
      delete magn_cd_;
      delete magn_cd_dx_;
      delete magn_cd_dy_;
      delete magn_cdd_;
      delete phase_cd_;
    }

  protected:
    // ********** methods **********
    void init(const Image<pixelIntensity>* img);
    Image<float>* getMagnitude(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy);
    Image<float>* getPhase(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy);
};

class Pyramid{
  public:
    // ********** members **********
    int levels_;
    std::vector< PyramidLevel* > pyramid_levels_;

    // ********** constructor **********
    Pyramid(Camera* cam, const int levels):
    levels_(levels){
      init(levels, cam->image_intensity_);
    }

    ~Pyramid(){
      for (PyramidLevel* pyr_lvl : pyramid_levels_)
        delete pyr_lvl;
    }

    // ********** methods **********
    void init(const int levels, const Image<pixelIntensity>* image_intensity);
    Image<pixelIntensity>* getC(int level);
    Image<pixelIntensity>* getCDX(int level);
    Image<pixelIntensity>* getCDY(int level);
    Image<pixelIntensity>* getMagn(int level);
    Image<pixelIntensity>* getMagnDX(int level);
    Image<pixelIntensity>* getMagnDY(int level);
    Image<pixelIntensity>* getMagn2(int level);
    Image<pixelIntensity>* getPhase(int level);


  protected:

};
