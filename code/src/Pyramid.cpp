#include "Pyramid.h"
#include "CameraForMapping.h"

void PyramidLevel::init(const Image<pixelIntensity>* img){
  c_= new Image<pixelIntensity>(img);
  c_dx_=c_->compute_sobel_x();
  c_dy_=c_->compute_sobel_y();
  magn_cd_=getMagnitude(c_dx_,c_dy_);
  magn_cd_dx_=magn_cd_->compute_sobel_x();
  magn_cd_dy_=magn_cd_->compute_sobel_y();
  phase_cd_=getPhase(c_dx_,c_dy_);
}


Image<float>* PyramidLevel::getMagnitude(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy )
{
  Image<pixelIntensity>* magn (new Image<pixelIntensity>("magn_"+dx->name_+" "+dy->name_));
  cv::magnitude(dx->image_, dy->image_, magn->image_);

  return magn;
}

Image<float>* PyramidLevel::getPhase(Image<pixelIntensity>* dx, Image<pixelIntensity>* dy )
{
  Image<pixelIntensity>* phase (new Image<pixelIntensity>("phase_"+dx->name_+" "+dy->name_));
  cv::phase(dx->image_, dy->image_, phase->image_);

  return phase;
}


void Pyramid::init(const int levels, const Image<pixelIntensity>* image_intensity){
  assert(levels>0);

  // clone image
  Image<pixelIntensity>* img (new Image<pixelIntensity>(image_intensity)) ;

  for(int level=0; level<levels_; level++){
    pyramid_levels_.push_back( new PyramidLevel(img) );

    if(level!=levels_)
      cv::resize(img->image_, img->image_, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR );
  }
}

Image<pixelIntensity>* Pyramid::getC(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->c_;
}

Image<pixelIntensity>* Pyramid::getCDX(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->c_dx_;
}

Image<pixelIntensity>* Pyramid::getCDY(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->c_dy_;
}

Image<pixelIntensity>* Pyramid::getMagn(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cd_;
}

Image<pixelIntensity>* Pyramid::getMagnDX(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cd_dx_;
}

Image<pixelIntensity>* Pyramid::getMagnDY(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->magn_cd_dy_;
}

Image<pixelIntensity>* Pyramid::getPhase(int level){
  assert(level>=0 && level<pyramid_levels_.size());
  return pyramid_levels_[level]->phase_cd_;
}
