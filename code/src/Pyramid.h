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
    PyramidLevel(Image<pixelIntensity>* img,
                 Image<pixelIntensity>* c_dx, Image<pixelIntensity>* c_dy,
                 Image<pixelIntensity>* cd_magn,
                 Image<pixelIntensity>* c_dx2, Image<pixelIntensity>* c_dy2,
                 Image<pixelIntensity>* cd_magn2,
                 Image<pixelIntensity>* cd_phase){
      init(img, c_dx,c_dy,cd_magn,c_dx2,c_dy2,cd_magn2,cd_phase);
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
    void init(const Image<pixelIntensity>* img,
              const Image<pixelIntensity>* c_dx, const Image<pixelIntensity>* c_dy,
              const Image<pixelIntensity>* cd_magn,
              const Image<pixelIntensity>* c_dx2, const Image<pixelIntensity>* c_dy2,
              const Image<pixelIntensity>* cd_magn2,
              const Image<pixelIntensity>* cd_phase
    );
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
      init(levels, cam->image_intensity_, pyr_type);
    }

    ~Pyramid(){
      for (PyramidLevel* pyr_lvl : pyramid_levels_)
        delete pyr_lvl;
    }

    // ********** methods **********
    void init(const int levels, const Image<pixelIntensity>* image_intensity, bool pyr_type_);

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
